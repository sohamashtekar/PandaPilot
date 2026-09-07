from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, TI_STATE

VisualAlert = car.CarControl.HUDControl.VisualAlert


def _unwind_steer_to_zero(last, delta_down):
  if last > 0:
    return max(last - delta_down, 0)
  if last < 0:
    return min(last + delta_down, 0)
  return 0


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.ti_apply_steer_last = 0
    self.ti_knock_frames_left = CarControllerParams.TI_KNOCK_FRAMES
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_steer = 0
    ti_apply_steer = 0
    driver_tug = bool(CS.out.steeringPressed) or CS.ti_state == TI_STATE.DRIVER_OVER

    if CC.latActive:
      if CS.ti_present and driver_tug:
        # Quick unwind so the driver takes over without a dead-wheel jerk.
        # Resume still ramps up from whatever is left (DELTA_UP), not a snap.
        apply_steer = _unwind_steer_to_zero(self.apply_steer_last, CarControllerParams.TI_STEER_DELTA_DOWN_TUG)
        ti_apply_steer = _unwind_steer_to_zero(self.ti_apply_steer_last, CarControllerParams.TI_STEER_DELTA_DOWN_TUG)
      else:
        if CS.ti_present and CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * CarControllerParams.TI_STEER_MAX))
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                        CS.out.steeringTorque, CarControllerParams)

        # Stock camera spoof. With TI, cap this path at 600 as well (openpilot was 600+600).
        stock_steer_max = CarControllerParams.TI_STEER_MAX if CS.ti_present else CarControllerParams.STEER_MAX
        new_steer = int(round(CC.actuators.steer * stock_steer_max))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                       CS.out.steeringTorque, CarControllerParams)

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.RESUME))

    # Fault/disengage still snap to 0. Tug unwind keeps last so resume is continuous.
    self.apply_steer_last = apply_steer
    self.ti_apply_steer_last = ti_apply_steer

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # CAM_LKAS2 wakes a silent TI. Continuous 100 Hz with no interceptor previously
    # latched camera ERR_BIT_1 on this C2, so only knock briefly at startup, then
    # keep sending if a heartbeat was seen.
    send_ti = CS.ti_present or CS.ti_was_present
    if self.ti_knock_frames_left > 0:
      send_ti = True
      self.ti_knock_frames_left -= 1
      if CS.ti_present:
        self.ti_knock_frames_left = 0
    if send_ti:
      can_sends.extend(mazdacan.create_ti_steering_control(self.packer, self.CP.carFingerprint,
                                                           self.frame, ti_apply_steer))
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP.carFingerprint,
                                                      self.frame, apply_steer, CS.cam_lkas))

    new_actuators = CC.actuators.copy()
    if CS.ti_present:
      new_actuators.steer = ti_apply_steer / CarControllerParams.TI_STEER_MAX
      new_actuators.steerOutputCan = ti_apply_steer
    else:
      new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
      new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
