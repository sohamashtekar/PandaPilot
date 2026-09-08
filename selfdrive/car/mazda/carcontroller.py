from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, TI_STATE, mazda_ti_mode, TIModeStockLimits

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
    self.ti_mode = mazda_ti_mode(CP)
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    apply_steer = 0
    ti_apply_steer = 0
    stock_limits = TIModeStockLimits if self.ti_mode else CarControllerParams
    driver_tug = bool(CS.out.steeringPressed) or CS.ti_state == TI_STATE.DRIVER_OVER

    if CC.latActive:
      if CS.ti_present and driver_tug:
        # Quick unwind so the driver takes over without a dead-wheel jerk.
        # Resume still ramps up from whatever is left (DELTA_UP), not a snap.
        apply_steer = _unwind_steer_to_zero(self.apply_steer_last, CarControllerParams.TI_STEER_DELTA_DOWN_TUG)
        ti_apply_steer = _unwind_steer_to_zero(self.ti_apply_steer_last, CarControllerParams.TI_STEER_DELTA_DOWN_TUG)
      else:
        # dp-newcan: independent CAM_LKAS 600 + CAM_LKAS2 600 while TI is in RUN.
        if CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * CarControllerParams.TI_STEER_MAX))
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                        CS.out.steeringTorque, CarControllerParams)

        new_steer = int(round(CC.actuators.steer * stock_limits.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                       CS.out.steeringTorque, stock_limits)

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

    # dp-newcan: always send CAM_LKAS2 when TI mode is on (torque 0 until RUN).
    # Off: do not send 0x249 so a C2 without an interceptor stays stock.
    if self.ti_mode:
      can_sends.extend(mazdacan.create_ti_steering_control(self.packer, self.CP.carFingerprint,
                                                           self.frame, ti_apply_steer))
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP.carFingerprint,
                                                      self.frame, apply_steer, CS.cam_lkas))

    new_actuators = CC.actuators.copy()
    new_actuators.steer = apply_steer / stock_limits.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends
