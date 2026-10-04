# VectorNav VN-200 GNSS/INS: hardware reference for an external error-state EKF on a sensor-limited AUV

> **Source-access caveat (read first).** The research sandbox's egress proxy blocked every direct fetch of vectornav.com, the UM004 and UM2000 PDFs on navtechgps.com, geo-matching.com and metromatics.com.au, and archive.org. So **no official VectorNav PDF was read first-hand.** The evidence comes from three places:
> 1. Web-search result snippets that quote the official datasheets and manuals. These came through a search summarizer and are flagged where single-sourced.
> 2. Third-party open-source code that copies the official manual text into doc comments: UniStuttgart-INS/INSTINCT, the dawonn/vectornav ROS driver, and VectorNav's own vnproglib/SDK headers mirrored on GitHub.
> 3. Reasoning, kept in "Inferences".
>
> Field **names** are confirmed from vnproglib's `types.h`, which is VectorNav's own library. Field **descriptions and units** are confirmed from the INSTINCT doc comments, which quote the manual verbatim. Every numeric spec below should be checked against the official datasheet PDF before it goes into filter tuning.

## 1. Which manual and revision is current?

### Takeaway
UM004 is the old document number. The current VN-200 user manual has been reissued as **"VN-200 GNSS/INS User Manual" UM2000 (rev 2)**. The last widely mirrored UM004 is **Firmware v1.1.0.0, Document Revision 2.22**. The current datasheets are split by package: **Rugged = DS200-CR-33 (Hardware v3.3)** and **SMD = DS200-SMD-30 (Hardware v3.0)**.

### Cited Findings
- A distributor hosts a document titled "PRODUCT SPECIFICATION VN-200 GNSS/INS User Manual" under the file name `vn200-user-manual-um2000-r2.pdf`. — [Metromatics (search result)](https://metromatics.com.au/wp-content/uploads/2025/12/vn200-user-manual-um2000-r2.pdf)
- Older manual: "UM004 1 Firmware v1.1.0.0 Document Revision 2.22 VN-200 User Manual". — [geo-matching mirror (search result)](https://cdn.geo-matching.com/ZprPqORL.pdf)
- An even older UM004, dated 08/05/14, exists. — [NavtechGPS mirror (search result)](https://www.navtechgps.com/wp-content/uploads/assets/1/7/VN200UserManual_UM004_080514.pdf)
- Datasheets: "VN-200 RUGGED GNSS/INS Sensor Datasheet (Hardware v3.3)" `DS200-CR-33-R1`, and "VN-200 SMD GNSS/INS Sensor Datasheet (Hardware v3.0)" `DS200-SMD-30-R1`. — [Rugged DS](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf); [SMD DS](https://metromatics.com.au/wp-content/uploads/2025/12/VN200SMD-Datasheet-v3.0-DS200-SMD-30-R1.pdf)
- The ROS driver's InsStatus message cites "UM005 - 10.2.2". UM005 is a different VectorNav manual number (not VN-200-specific). Driver comments therefore mix manuals. — [dawonn/vectornav InsStatus.msg](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav_msgs/msg/InsStatus.msg)

### Inferences
- The project should cite **UM2000 r2** plus the datasheet matching the actual hardware revision (rugged v3.3 or SMD v3.0). Check the hardware and firmware revision on the unit with registers 2 (HW revision) and 4 (firmware), as listed in a VectorNav driver plan derived from the manual. — [nekosaif/mtrtk plan doc](https://github.com/nekosaif/mtrtk/blob/main/docs/superpowers/plans/2026-09-19-phase10-ins-drivers.md)
- Older UM004 revisions used "GPS" naming (GpsFix, GpsCompass). Newer documents use "GNSS". The bit positions appear unchanged.

### Gaps
- I could not open UM2000 r2 to confirm its section numbering, or whether field text changed from UM004 r2.22.

## 2. Binary output field lists: Group 3 (IMU), Group 5 (Attitude), Group 6 (INS)

### Takeaway
The field names asked about are exactly right. Bit order is confirmed from VectorNav's vnproglib, and units and frames come from manual-derived doc comments. The key distinctions:
- **Uncomp\*** fields carry only the factory static calibration. They are not corrected by the onboard Kalman-filter biases.
- **Mag/Accel/AngularRate** carry the onboard filter's dynamic bias compensation.
- **DeltaTheta/DeltaVel** are coning- and sculling-integrated at the 800 Hz IMU rate.
- The **quaternion is scalar-last**, body with respect to NED.
- The **DCM is column-major and maps NED to body**.
- Yaw/pitch/roll is a **3-2-1 sequence, body with respect to NED**.

### Cited Findings
**Group 3: IMU.** Bit masks from vnproglib:
- ImuStatus 0x0001
- UncompMag 0x0002
- UncompAccel 0x0004
- UncompGyro 0x0008
- Temp 0x0010
- Pres 0x0020
- DeltaTheta 0x0040
- DeltaVel 0x0080
- Mag 0x0100
- Accel 0x0200
- AngularRate 0x0400
- SensSat 0x0800. The ROS driver author notes that "SENSSAT exists in the header, but not the manual".

Sources: [vnproglib types.h (dawonn mirror)](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav/vnproglib-1.2.0.0/cpp/include/vn/types.h); [ImuGroup.msg](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav_msgs/msg/ImuGroup.msg)

Group 3 fields, quoting the manual-derived doc comments in [INSTINCT ImuOutputs.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/BinaryOutputs/ImuOutputs.hpp):
- **ImuStatus**: "Reserved for future use… will always report 0."
- **UncompMag**: Gauss, body frame. "compensated by the static calibration (individual factory calibration stored in flash), and the user compensation, however it is not compensated by the onboard Hard/Soft Iron estimator."
- **UncompAccel**: m/s^2, body frame. "compensated by the static calibration… however it is not compensated by any dynamic calibration such as bias compensation from the onboard INS Kalman filter."
- **UncompGyro**: rad/s, body frame. Static calibration only. "not compensated by any dynamic calibration such as the bias compensation from the onboard AHRS/INS Kalman filters."
- **Temp**: "IMU temperature measured in units of Celsius."
- **Pres**: "The IMU pressure measured in kilopascals. This is an absolute pressure measurement. Typical pressure at sea level would be around 100 kPa."
- **DeltaTheta**: delta time plus a 3-axis delta angle, in degrees, "calculated via onboard coning and sculling integration at full IMU rate (800Hz)." The doc-comment text was paraphrased by the fetch tool. Units are given as degrees.
- **DeltaVel**: m/s. "the delta velocity incurred due to motion, since the last time the values were output by the device… calculated based upon the onboard conning and sculling integration performed onboard the sensor at the IMU sampling rate (nominally 800Hz)."
- **Mag**: Gauss, body frame. Includes static calibration, user calibration and the "dynamic calibration from the onboard Hard/Soft Iron estimator."
- **Accel**: m/s^2, body frame. Includes static calibration, user calibration and "the dynamic bias compensation from the onboard INS Kalman filter."
- **AngularRate**: rad/s, body frame. Includes static calibration, user calibration and "the dynamic bias compensation from the onboard INS Kalman filter."

Other Group 3 sources:
- The ROS message packs DeltaTheta as `deltatheta_time` (float32) plus `deltatheta_dtheta` (Vector3). That is 4 floats in total: dt, then dθx, dθy, dθz. — [ImuGroup.msg](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav_msgs/msg/ImuGroup.msg)
- The delta-theta/delta-velocity integration is configurable through register **DeltaThetaVelConfig (ID 82)**. Source: VectorNav SDK register docs, summarized in [GSO-soslab vectornav_registers.md](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md). The register has these options:
  - `integrationFrame`: Body or NED
  - `gyroCompensation`: None or Bias
  - `accelCompensation`: None, Gravity, Bias, or BiasAndGravity
  - `earthRateCompensation`: None, GyroRate, CoriolisAccel, or RateAndCoriolis
- The register DeltaThetaVelocity (ID 80) "contains the output values of the onboard coning and sculling algorithm". — [VectorNav SDK Registers.hpp (mirror)](https://github.com/dotysan/vectornav-sdk/blob/add7bddaf6bcd773ad4acd497fd7df31263e7865/cpp/include/vectornav/Interface/Registers.hpp)

**Group 5: Attitude.** Bit masks:
- VpeStatus 0x0001
- YawPitchRoll 0x0002
- Quaternion 0x0004
- DCM 0x0008
- MagNed 0x0010
- AccelNed 0x0020
- LinearAccelBody 0x0040
- LinearAccelNed 0x0080
- YprU 0x0100
- vnproglib also defines Heave 0x1000.

Source: [vnproglib types.h](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav/vnproglib-1.2.0.0/cpp/include/vn/types.h)

Group 5 fields, quoting [INSTINCT AttitudeOutputs.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/BinaryOutputs/AttitudeOutputs.hpp):
- **YawPitchRoll**: degrees, "3,2,1 Euler angle sequence describing the body frame with respect to the local North East Down (NED) frame". Ranges: yaw ±180°, pitch ±90°, roll ±180°.
- **Quaternion**: "The last term is the scalar value. The attitude is given as the body frame with respect to the local North East Down (NED) frame." That is, the order is [x, y, z, w]. The VectorNav SDK register also orders it `quatX, quatY, quatZ, quatS`. — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- **DCM**: "given in column major order. The DCM maps vectors from the North East Down (NED) frame into the body frame."
- **MagNed**: Gauss, NED frame. Includes static and dynamic calibration.
- **AccelNed**: m/s^2, with gravity, NED frame. Bias-compensated by the INS filter.
- **LinearAccelBody**: m/s^2, without gravity, body frame. Bias-compensated, with gravity removed.
- **LinearAccelNed**: as LinearAccelBody, but in the NED frame.
- **YprU**: 1σ attitude uncertainty in degrees. It is "not valid when the INS Scenario mode… is set to AHRS mode."

**Group 6: INS.** Bit masks:
- InsStatus 0x0001
- PosLla 0x0002
- PosEcef 0x0004
- VelBody 0x0008
- VelNed 0x0010
- VelEcef 0x0020
- MagEcef 0x0040
- AccelEcef 0x0080
- LinearAccelEcef 0x0100
- PosU 0x0200
- VelU 0x0400

Sources: [vnproglib types.h](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav/vnproglib-1.2.0.0/cpp/include/vn/types.h); [InsGroup.msg](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav_msgs/msg/InsGroup.msg)

Group 6 fields, quoting [INSTINCT InsOutputs.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/BinaryOutputs/InsOutputs.hpp):
- **PosLla**: [deg, deg, m].
- **PosEcef**: m.
- **VelBody**: "estimated velocity in the body frame, given in m/s".
- **VelNed** and **VelEcef**: m/s.
- **MagEcef**: compensated magnetometer reading, Gauss.
- **AccelEcef**: with gravity, bias-compensated. "should be nominally equivalent to the gravity reference vector" when the unit is stationary and the filter is tracking.
- **LinearAccelEcef**: without gravity. Reads nominally 0 when stationary.
- **PosU**: 1σ position uncertainty, m.
- **VelU**: 1σ velocity uncertainty, m/s.

Field sizes and types, from a driver plan derived from the manual (secondary source):
- Group 3: `Temp f °C, Pres f kPa, DeltaTheta 4f, DeltaVel 3f, Accel 3f m/s², AngularRate 3f rad/s`
- Group 5: `YprU 3f ° (1σ)`
- Group 6: `PosLla 3d, PosEcef 3d, VelBody 3f … PosU f (m, 1σ), VelU f`
- The output rate is set by a divisor of the 800 Hz IMU rate. Valid rates in Hz are {1, 2, 4, 5, 8, 10, 16, 20, 25, 32, 40, 50, 80, 100, 160, 200, 400, 800}.

Source: [nekosaif/mtrtk plan](https://github.com/nekosaif/mtrtk/blob/main/docs/superpowers/plans/2026-09-19-phase10-ins-drivers.md)

**Maximum rates:** "400 Hz Navigation data; 800 Hz IMU data". — [VN-200 product brief (search result)](https://cornestech.co.jp/wp-content/uploads/2022/01/VectorNav_VN-200_product_brief_CTL.pdf). VectorNav also states that compensated inertial and navigation outputs come "at rates between 400 and 800 Hz". — [VectorNav INS solutions page (search result)](https://www.vectornav.com/solutions/ins)

### Inferences
- **For the external error-state EKF:**
  - The cleanest propagation input is either UncompGyro/UncompAccel at up to 800 Hz, or DeltaTheta/DeltaVel at a lower rate with no loss of integration fidelity.
  - Set DeltaThetaVelConfig so the deltas are **body-frame with no bias, gravity or earth-rate compensation**. Then the deltas are raw increments, and your own filter estimates the biases.
  - Avoid Accel/AngularRate as propagation inputs. They already contain the internal filter's bias estimate, which is correlated with the internal filter. Feeding them in breaks the independence assumptions of your EKF, and while the INS is in mode 3 (or under AHRS behaviour), the internal bias estimates may wander or freeze.
- Remember the scalar-last quaternion convention when converting to Hamilton/Eigen `(w, x, y, z)`.
- The DCM is NED→body, i.e. C_b^n transposed. Check it against your own C_b^n convention.
- The VN-200 body frame is the sensor frame printed on the housing unless a Reference Frame Rotation (register 26) is applied. That register is listed in [the mtrtk plan](https://github.com/nekosaif/mtrtk/blob/main/docs/superpowers/plans/2026-09-19-phase10-ins-drivers.md).

### Gaps
- The exact DeltaTheta unit (degrees, per the INSTINCT doc comment) and the reset semantics should be checked in UM2000. The DeltaTheta text above was paraphrased by the fetch tool, not quoted.
- The SensSat bit definitions were not found.
- Whether VelBody is expressed at the IMU or at the INS reference point (register InsRefOffset) is unconfirmed.

## 3. InsStatus bits and INS behaviour when GNSS is lost; VPE heading modes and magnetometer handling

### Takeaway
InsStatus bits 0-1 encode the filter mode:
- 0 = Not tracking
- 1 = Aligning
- 2 = Tracking
- 3 = **Loss of GNSS**: "A GNSS outage has lasted more than 45 seconds. The INS Filter will no longer update the position and velocity outputs, but the attitude remains valid."

So the **internal INS dead-reckons for only about 45 s**. After that, position and velocity freeze, and only attitude stays valid. For a submerged AUV, the VN-200's own position and velocity outputs are useless after 45 s underwater. The project needs its own filter.

### Cited Findings
- InsStatus bitfield, quoting the manual text in [INSTINCT VectorNavTypes.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/VectorNavTypes.hpp):
  - Bits 0+1, Mode:
    - 0 = Not tracking.
    - 1 = Aligning. "INS Filter is dynamically aligning… if the INS Filter drops from INS Mode 2 back down to 1, the attitude uncertainty has increased above 2 degrees."
    - 2 = Tracking. "operating within specification".
    - 3 = Loss of GNSS. ">45 s; no longer update the position and velocity outputs, but the attitude remains valid."
  - Bit 2: GpsFix.
  - Bit 4: IMU Error.
  - Bit 5: Mag/Pres Error.
  - Bit 6: GNSS Error.
  - Bit 8: GpsHeadingIns.
  - Bit 9: GpsCompass.
  - **Caveat:** the mode-0 text mentions "GNSS Compass", so this wording was taken from the dual-antenna VN-300/VN-310 manual. The 45 s / mode-3 text is the same idea the ROS driver uses (`MODE_NO_GPS = 3`). — [dawonn InsStatus.msg](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav_msgs/msg/InsStatus.msg)
- An independent driver plan lists InsStatus as: bits 0-1 mode (0 not tracking, 1 aligning / insufficient dynamic motion, 2 tracking), bit 2 GpsFix, bit 3 time error, bit 4 IMU error, bit 5 mag/pres error, bit 6 GPS error, bit 8 GpsHeadingIns, bit 9 GpsCompass. — [mtrtk plan](https://github.com/nekosaif/mtrtk/blob/main/docs/superpowers/plans/2026-09-19-phase10-ins-drivers.md)
- The older vnproglib enum names bit 0 "SUFFICIENT_DYNAMIC_MOTION", bit 1 "TRACKING", 0x04 GPS_FIX, 0x08 TIME_ERROR ("INS filter loop exceeds 5 ms"), 0x10 IMU_ERROR, 0x20 MAG_PRES_ERROR ("Magnetometer or pressure sensor error"), and 0x40 GPS_ERROR. — [vnproglib types.h](https://raw.githubusercontent.com/dawonn/vectornav/ros2/vectornav/vnproglib-1.2.0.0/cpp/include/vn/types.h)
- The VectorNav SDK describes `gnssErr` as "High if GNSS communication error is detected or if no valid PPS signal is received". — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- The product brief says that "after loss of GNSS signal, the typical rate of growth in error of position estimates is 3.0 cm/s²". This is a single search-summarizer snippet and is unverified. — [VN-200 product brief (search result)](https://www.navtechgps.com/wp-content/uploads/VN200_ProductBrief_DS.pdf)
- The same brief says the INS Kalman filter uses accelerometer, gyroscope and GNSS data, plus the magnetometer at startup. — [VN-200 product brief (search result)](https://www.navtechgps.com/wp-content/uploads/VN200_ProductBrief_DS.pdf)
- VectorNav: "Tactical-grade sensors (VN-210 / VN-310) provide significantly longer and more accurate GNSS-denied performance compared to Industrial sensors". — [VectorNav INS page (search result)](https://www.vectornav.com/solutions/ins)
- **InsBasicConfig (register 67)** `scenario` values are Ahrs, GnssInsWithPressure, GnssInsNoPressure, DualGnssNoPressure and DualGnssWithPressure. The register also has `ahrsAiding` (Disable/Enable) and `estBaseline`. — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- **VpeBasicControl (register 35)** has these settings:
  - `headingMode`: Absolute, Relative, Indoor
  - `filteringMode`: Unfiltered, AdaptivelyFiltered
  - `tuningMode`: Static, Adaptive
  - Tuning registers VpeMagBasicTuning (36) and VpeAccelBasicTuning (38) take base, adaptive tuning and adaptive filtering values per axis.

  Source: [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- **Hard/soft iron:** RealTimeHsiControl (register 44) has `mode` (Off/Run/Reset), `applyCompensation` and `convergeRate`. MagCal (register 23) holds a 3x3 gain matrix plus a 3-element bias. — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- VpeStatus bits:
  - 0-1: AttitudeQuality (0 Excellent … 3 Not tracking)
  - 2: GyroSaturation
  - 3: GyroSaturationRecovery
  - 4-5: MagDisturbance
  - 6: MagSaturation
  - 7-8: AccDisturbance
  - 9: AccSaturation
  - 11: KnownMagDisturbance
  - 12: KnownAccelDisturbance

  Source: [INSTINCT VectorNavTypes.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/VectorNavTypes.hpp)

### Inferences
- In mode 3 the VN-200 still provides attitude, including heading. Heading is unobservable without GNSS motion or magnetometer aiding, so it will drift at the gyro bias rate, or be pulled toward magnetic heading if the magnetometer is used.
- A metal-and-thruster AUV will cause magnetic disturbance. The VpeStatus MagDisturbance bits are a useful health flag.
- After the vehicle resurfaces, expect a transition back through mode 1 to mode 2 as GNSS returns. Use the mode bits as the gate for accepting VN-200 PosLla/VelNed as measurements in your EKF.
- Choosing "relative" heading mode, or switching the scenario to AHRS while submerged, are possible configurations. Neither is validated for this use.
- The exact behaviour of the INS-to-AHRS transition is not documented in the material I could reach. From what was found, it does **not** automatically switch to the AHRS scenario. It stays in INS mode 3 with attitude still valid.

### Gaps
- I could not confirm UM2000's VN-200-specific wording for mode 0/1, or whether the 45 s threshold is the same in current VN-200 firmware. The text found is VN-310-flavoured.
- I could not confirm the official meaning of "Relative" and "Indoor" heading modes. The fetch-tool paraphrase was "heading relative to initial orientation" and "magnetic model disabled", which is unverified.
- **External aiding.** The SDK lists **VelAidingMeas (register 50, velocityX/Y/Z)** and **VelAidingControl (register 51, velAidEnable, velUncertTuning)**. — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
  - The frame of the input, and whether the VN-200 INS filter (as opposed to only the VPE/AHRS acceleration compensation) actually fuses it, were **not confirmed** from an official document.
  - In older VN-100/VN-200 manuals, registers 50/51 were called "Velocity Compensation Measurement/Control". From memory, these were documented as compensating the AHRS accelerometer for dynamic acceleration, not as a navigation aid. This is **unverified** and must be checked in UM2000 before relying on it to feed thrust-model velocity into the internal INS.
- I found no VectorNav application note on underwater use, or on GNSS-outage tuning specific to the VN-200.

## 4. Barometric pressure sensor ("Pres"): present? usable underwater?

### Takeaway
Yes. The VN-200 has an onboard absolute barometric pressure sensor. It outputs Pres in **kPa**, absolute, "typical… sea level… around 100 kPa". The datasheet range is quoted as **10 to 1200 mbar** (1 to 120 kPa), which is a barometer range. That range saturates at roughly 2 m of water depth, so **it cannot work as a depth sensor**. If the VN-200 sits in a sealed dry housing, Pres measures the housing's internal air pressure. That pressure changes with temperature and leaks, not with depth.

### Cited Findings
- Pres: "The IMU pressure measured in kilopascals. This is an absolute pressure measurement. Typical pressure at sea level would be around 100 kPa." — [INSTINCT ImuOutputs.hpp (manual text)](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/BinaryOutputs/ImuOutputs.hpp)
- The barometer range is "10 to 1200 mbar". This is a single search-summarizer snippet from the datasheet search. — [VN-200 datasheet search result](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf)
- The INS scenarios include "GnssInsWithPressure" and "GnssInsNoPressure", so the pressure sensor can be used as an altitude aid. — [SDK register docs (soslab)](https://raw.githubusercontent.com/GSO-soslab/vectornav_python/main/vectornav_registers.md)
- The InsStatus "Mag/Pres Error" bit is "High if Magnetometer or Pressure sensor error is detected". — [INSTINCT VectorNavTypes.hpp](https://raw.githubusercontent.com/UniStuttgart-INS/INSTINCT/main/src/util/Vendor/VectorNav/VectorNavTypes.hpp)

### Inferences
- 1200 mbar is 120 kPa, only about 20 kPa above sea-level atmosphere. Seawater adds about 10 kPa per metre, so even if the sensor were directly exposed to water, it would saturate at roughly 2 m depth. Exposing it to water would also damage it.
- In a sealed housing, Pres reads the internal gas pressure, roughly p ∝ T per the ideal gas law. It is not a depth signal. It may serve as a **housing leak and over-temperature health monitor**, by detecting sudden pressure rises.
- Set the INS scenario to **GnssInsNoPressure**. Otherwise the internal filter may fuse housing pressure as barometric altitude.
- **Depth is therefore unobservable from the VN-200 alone.** Only inertial vertical integration is available, which diverges quickly, plus any model-based heave/buoyancy constraint in the user's filter.

### Gaps
- No official VectorNav statement on underwater or sealed-housing use of the pressure sensor was found.
- Pressure resolution and noise specs were not retrieved.

## 5. Datasheet specs for filter tuning

### Takeaway
VN-200 (industrial-grade MEMS) headline numbers:
- Gyro: in-run bias < 10 °/hr (5 °/hr typical), noise density 0.0035 °/s/√Hz (≈ 0.21 °/√hr ARW), range ±2000 °/s, bandwidth 265 Hz
- Accelerometer: in-run bias < 0.04 mg, noise density 0.14 mg/√Hz
- Magnetometer: noise density 140 µGauss/√Hz
- Rates: IMU 800 Hz, navigation 400 Hz
- INS accuracy: heading 0.2°, pitch/roll 0.03°, horizontal position 1.0 m RMS, vertical 1.5 m RMS, velocity < 0.05 m/s

All of these come from search snippets of the official datasheets and briefs. **Each needs confirming against the PDF for the user's exact variant (Rugged v3.3 vs SMD v3.0).**

### Cited Findings
- Gyro: in-run bias stability "< 10°/hr (5°/hr typ.)", noise density "0.0035 °/s √Hz", bandwidth 265 Hz, cross-axis "< 0.05°". This is a search snippet. The search hit both the Rugged v3.3 and SMD v3.0 datasheets, and it is not certain which one it came from. — [VN-200 Rugged DS v3.3](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf); [VN-200 SMD DS v3.0](https://metromatics.com.au/wp-content/uploads/2025/12/VN200SMD-Datasheet-v3.0-DS200-SMD-30-R1.pdf)
- Gyro range "±2000 °/s". — [datasheet search result](https://metromatics.com.au/wp-content/uploads/2025/12/VN200SMD-Datasheet-v3.0-DS200-SMD-30-R1.pdf)
- Accelerometer: in-run bias stability "< 0.04 mg", noise density "< 0.14 mg/√Hz". Magnetometer noise density "140 μGauss/√Hz". This came from the rugged-datasheet search. — [VN-200 Rugged DS v3.3](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf)
- All sensors are individually calibrated over -40 °C to +85 °C. — [same](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf)
- INS (product brief):
  - Dynamic heading 0.2°
  - Dynamic pitch/roll 0.03°
  - Horizontal position 1.0 m RMS
  - Vertical position 1.5 m RMS
  - Velocity < 0.05 m/s
  - Static pitch/roll 0.5° RMS
  - Magnetic heading 2.0° RMS
  - 400 Hz navigation / 800 Hz IMU

  Source: [VN-200 product brief (CornesTech mirror)](https://cornestech.co.jp/wp-content/uploads/2022/01/VectorNav_VN-200_product_brief_CTL.pdf)
  - **Conflict:** an older NavtechGPS brief advertises "Dynamic Accuracy better than 0.25° in Pitch/Roll, 0.75° in Heading". That is an older hardware generation. — [NavtechGPS old brief](https://www.navtechgps.com/wp-content/uploads/assets/1/7/VN200_ProductBrief_DS.pdf)
- The **rugged** variant is in a "clamshell precision anodized aluminum enclosure". The **SMD** variant is the "world's first single packaged SMD GNSS/INS". — [rugged DS search result](https://metromatics.com.au/wp-content/uploads/2025/12/VN200CR-Datasheet-v3.3-DS200-CR-33-R1.pdf); [VectorNav product page title](https://www.vectornav.com/products/detail/vn-200)

### Inferences
- Unit conversions for the EKF:
  - Gyro white noise: 0.0035 °/s/√Hz = 6.1e-5 rad/s/√Hz. Converting to angle random walk: 0.0035 × 60 ≈ **0.21 °/√hr**.
  - Accel white noise: 0.14 mg/√Hz ≈ **1.37e-3 m/s²/√Hz**, which is a VRW of about 0.082 m/s/√hr.
  - Gyro bias instability: 5-10 °/hr ≈ 2.4e-5 to 4.8e-5 rad/s.
  - Accel bias instability: 0.04 mg ≈ 3.9e-4 m/s².
  - These are lab Allan-variance figures. Inflate them by about 2-10x for a vibrating, temperature-varying AUV, and validate with your own Allan variance test.
- Free-inertial position error with a 0.04 mg accel bias grows as ½·b·t² ≈ 0.5 × 3.9e-4 × t². That gives about 0.7 m after 60 s and about 70 m after 600 s, from accel bias alone. Attitude-error-induced gravity leakage makes it worse. This is why the thrust-model velocity aid is essential.

### Gaps
- Unverified:
  - accelerometer range (±16 g is commonly cited but was not confirmed here)
  - magnetometer range
  - gyro and accel bias repeatability
  - g-sensitivity
  - GNSS receiver specs (channels, constellations, CEP, TTFF, velocity accuracy)
  - pressure noise
- Datasheet PDFs could not be opened, so I could not confirm which numbers differ between the SMD v3.0 and Rugged v3.3 datasheets.
