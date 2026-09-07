import time

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.swaglog import cloudlog
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.mazda.values import DBC, LKAS_LIMITS, GEN1, TI_STATE, CarControllerParams


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.low_speed_alert = False
    self.lkas_allowed_speed = False
    self.lkas_disabled = False

    self.ti_present = False
    self.ti_was_present = False
    self.ti_lkas_allowed = False
    self.ti_fault = False
    self.ti_ramp_down = False
    self.ti_version = 1
    self.ti_state = TI_STATE.OFF
    self.ti_violation = 0
    self.ti_error = 0
    self.ti_last_seen = 0.0
    self.ti_driver_torque = 0.0
    self.ti_steering_pressed = False

  def update(self, cp, cp_cam, cp_body=None):
    ret = car.CarState.new_message()
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph < .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    stock_steer_torque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]

    now = time.monotonic()
    self._update_ti(cp_body, now)

    if self.ti_present:
      ret.steeringTorque = self.ti_driver_torque
      if abs(ret.steeringTorque) >= CarControllerParams.TI_STEER_THRESHOLD:
        self.ti_steering_pressed = True
      elif abs(ret.steeringTorque) < CarControllerParams.TI_STEER_THRESHOLD_RELEASE:
        self.ti_steering_pressed = False
      ret.steeringPressed = self.ti_steering_pressed
    else:
      self.ti_steering_pressed = False
      ret.steeringTorque = stock_steer_torque
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gas = cp.vl["ENGINE_DATA"]["PEDAL_GAS"]
    ret.gasPressed = ret.gas > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # Do not wait for LKAS_BLOCK to clear: stock EPS asserts it below ~45 kph.
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    # LKAS_BLOCK is stock camera LKAS. Below ~45 kph it is normally set; treating
    # it as a fault would zero latActive so OP never sends (TI never sees a command).
    ret.steerFaultTemporary = (self.lkas_allowed_speed and lkas_blocked and
                               speed_kph >= LKAS_LIMITS.STOCK_DISABLE_SPEED)

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.lkas_disabled = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    # OP spoofs CAM_LKAS. Camera ERR_BIT_1 is not an EPS fault after intercept
    # and stayed stuck at 1 on this C2 after CAM_LKAS2, blocking engage.
    ret.steerFaultPermanent = False

    return ret

  def _update_ti(self, cp_body, now):
    self.ti_present = False
    self.ti_lkas_allowed = False

    ti_updated = False
    if cp_body is not None:
      try:
        ti_updated = len(cp_body.vl_all["TI_FEEDBACK"]["TI_TORQUE_SENSOR"]) > 0
      except (KeyError, TypeError, AttributeError):
        ti_updated = False

    if ti_updated:
      try:
        msg = cp_body.vl["TI_FEEDBACK"]
        sensor = msg["TI_TORQUE_SENSOR"]
        chksum = msg["CHKSUM"]
        # TI echoes torque in the checksum byte (raw byte0 == byte1).
        if sensor == chksum:
          first_heartbeat = self.ti_last_seen == 0.0
          self.ti_last_seen = now
          self.ti_driver_torque = sensor
          self.ti_version = int(msg["VERSION_NUMBER"])
          self.ti_state = int(msg["STATE"])
          self.ti_violation = int(msg["VIOL"])
          self.ti_error = int(msg["ERROR"])
          if self.ti_version > 1:
            self.ti_ramp_down = msg["RAMP_DOWN"] == 1
          else:
            self.ti_ramp_down = False
          if first_heartbeat:
            cloudlog.warning("TI_FEEDBACK heartbeat: state=%s version=%s torque=%s viol=%s err=%s" %
                             (self.ti_state, self.ti_version, self.ti_driver_torque,
                              self.ti_violation, self.ti_error))
      except (KeyError, TypeError, AttributeError):
        pass

    if self.ti_last_seen > 0.0 and (now - self.ti_last_seen) <= CarControllerParams.TI_HEARTBEAT_TIMEOUT:
      self.ti_present = True
      self.ti_was_present = True
      self.ti_lkas_allowed = (
        self.ti_state == TI_STATE.RUN and
        not self.ti_ramp_down and
        self.ti_error == 0 and
        self.ti_violation == 0
      )
      if self.ti_error != 0 or self.ti_violation != 0:
        self.ti_fault = True
      elif self.ti_lkas_allowed:
        self.ti_fault = False
    else:
      # Lost heartbeat: do not keep last TI torque as driver torque.
      self.ti_driver_torque = 0.0
      self.ti_lkas_allowed = False
      if self.ti_was_present:
        self.ti_fault = True
        if self.ti_last_seen > 0.0 and (now - self.ti_last_seen) > CarControllerParams.TI_FAULT_CLEAR_TIMEOUT:
          self.ti_was_present = False
          self.ti_fault = False

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("BLINK_INFO", 10),
      ("STEER", 67),
      ("STEER_RATE", 83),
      ("STEER_TORQUE", 83),
      ("WHEEL_SPEEDS", 100),
    ]

    if CP.carFingerprint in GEN1:
      messages += [
        ("ENGINE_DATA", 100),
        ("CRZ_CTRL", 50),
        ("CRZ_EVENTS", 50),
        ("CRZ_BTNS", 10),
        ("PEDALS", 50),
        ("BRAKE", 50),
        ("SEATBELT", 10),
        ("DOORS", 10),
        ("GEAR", 20),
        ("BSM", 10),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []

    if CP.carFingerprint in GEN1:
      messages += [
        # sig_address, frequency
        ("CAM_LANEINFO", 2),
        ("CAM_LKAS", 16),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

  @staticmethod
  def get_body_can_parser(CP):
    # Frequency 0: parse TI_FEEDBACK if present, never fail canValid when it is missing.
    messages = [
      ("TI_FEEDBACK", 0),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 1)
