# Offline Anomaly-Detection Evaluation

**Status:** Completed

## At a glance

- All 6 planned replays completed; no replay was marked failed or invalid.
- This is an **unlabeled** replay, so the reported detections are not measures of accuracy, precision, or recall.
- `local_text_cache_20` had the lower mean model-response latency (19.808 s versus 22.598 s for `local_text_cache_10`), a 2.790 s (12.3%) reduction.
- Both configurations produced positive detections in every replay. The 20-message cache was repeatable at 5 positives per replay; the 10-message cache varied slightly (6, 7, and 6 positives).
- Review the detailed decisions before treating the detection totals as operational results: several entries used the safe fallback after a malformed response, and the 20-message cache also recorded failed LLM calls.

### Reading this report

- A **positive decision** is a final decision marked `Anomaly: Yes`; it is not a verified anomaly because this evaluation has no labels.
- **Model latency** is reported only when a measured model response was available. An em dash (`—`) means no latency was recorded for that final decision.
- `None` in an aggregate severity or action distribution means those aggregate breakdowns were not collected, not that no severity or action occurred. Per-run breakdowns remain available below.
- The aggregate **Unparseable decisions** metric is reported by the evaluator as zero. Detailed rows can still identify malformed model output or an LLM-call failure when the evaluator produced a safe fallback decision; those rows are called out in the final-decision tables.

## Experiment summary

| Item | Value |
| --- | --- |
| Mode | unlabeled |
| Started (UTC) | 2026-09-02T18:55:35.692861+00:00 |
| Finished (UTC) | 2026-09-02T19:29:13.714288+00:00 |
| Playback rate | 1.0× |
| Executions | 6 attempted / 6 expected |
| Configuration file | /root/dev_ws/src/anomaly_detection/system_evaluation/offline_evaluation.yaml |
| Recordings location | /root/dev_ws/src/anomaly_detection/system_evaluation/bags/anomaly_20260727_183711 |
| Source revision | 03163ce8d9c49027d28908d7399a4d86ad1556fa |
| Report schema | 5 |

**Recordings**

| Bag | Recorded duration |
| --- | --- |
| /root/dev_ws/src/anomaly_detection/system_evaluation/bags/anomaly_20260727_183711 | 308.426 s |

> This was an unlabeled evaluation. Detection counts and configuration agreement describe replay behavior only; they are not accuracy measurements.

## Overall results

| Metric | Result |
| --- | --- |
| Expected executions | 6 |
| Completed executions | 6 |
| Failed executions | 0 |
| Invalid executions | 0 |

### Overall behavior and latency

| Metric | Result |
| --- | --- |
| Decisions measured for latency | 36 |
| Mean response latency | 21.435 s |
| Median response latency | 24.826 s |
| Minimum response latency | 3.918 s |
| Maximum response latency | 35.039 s |
| P90 response latency | 34.502 s |
| P95 response latency | 35.038 s |
| Positive decisions | 34 |
| Negative decisions | 35 |
| Unparseable decisions | 0 |
| Positive decision rate | 49.3% |
| Alerts | 33 |
| Severity distribution | None |
| Action distribution | None |
| LLM calls | 36 |
| Pending LLM calls at timeout | 0 |

## Configuration comparisons

### local_text_cache_10 vs. local_text_cache_20

Bag-level positive agreement: 100.0% across 3 shared run(s).

| Bag | Trial | Left positives | Right positives | Agrees |
| --- | --- | --- | --- | --- |
| anomaly_20260727_183711 | 1 | 6 | 5 | Yes |
| anomaly_20260727_183711 | 2 | 7 | 5 | Yes |
| anomaly_20260727_183711 | 3 | 6 | 5 | Yes |

## Configuration details

### local_text_cache_10

| Parameter | Value |
| --- | --- |
| Configuration hash | a4a9889040f3c1cb36ec609bcf8c3788b737cb302f80bf6b47819a380b511467 |
| API frequency | 10.000 s |
| Maximum cached messages | 10 |
| Maximum cache age | 30.000 s |
| Minimum trigger importance | warning |
| Model provider | gemma4 |
| Model | gemma4:12b-it-qat |
| Local model | Yes |
| Model token context | 3072 |
| Vision enabled | No |
| Image context enabled | No |
| Routine image frames | 1 |
| Maximum image frames | 2 |

#### Aggregate results

| Metric | Result |
| --- | --- |
| Decisions measured for latency | 21 |
| Mean response latency | 22.598 s |
| Median response latency | 24.893 s |
| Minimum response latency | 7.919 s |
| Maximum response latency | 34.495 s |
| P90 response latency | 32.152 s |
| P95 response latency | 32.764 s |
| Positive decisions | 19 |
| Negative decisions | 17 |
| Unparseable decisions | 0 |
| Positive decision rate | 52.8% |
| Alerts | 18 |
| Severity distribution | None |
| Action distribution | None |
| LLM calls | 21 |
| Pending LLM calls at timeout | 0 |

#### Repeatability across runs

| Metric | Runs | Mean | Std. dev. | Min | Max |
| --- | --- | --- | --- | --- | --- |
| Mean response latency (seconds) | 3 | 22.598 | 0.126 | 22.466 | 22.717 |
| Elapsed time (seconds) | 3 | 360.361 | 4.061 | 357.855 | 365.047 |
| Positive decisions | 3 | 6.333 | 0.577 | 6.000 | 7.000 |
| Negative decisions | 3 | 5.667 | 0.577 | 5.000 | 6.000 |
| Alerts | 3 | 6.000 | 0.000 | 6.000 | 6.000 |
| LLM calls | 3 | 7.000 | 0.000 | 7.000 | 7.000 |

**Bag-level trial consistency**

| Bag | Completed trials | Trials with positives | Positive outcome rate | Same outcome | Same positive count | Positive counts by trial |
| --- | --- | --- | --- | --- | --- | --- |
| anomaly_20260727_183711 | 3 | 3 | 100.0% | Yes | No | 6, 7, 6 |

#### Run: anomaly_20260727_183711 (trial 1)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T18:55:35.762623+00:00 |
| Finished (UTC) | 2026-09-02T19:01:40.809220+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 365.047 s |
| Decisions measured for latency | 7 |
| Mean response latency | 22.717 s |
| Median response latency | 25.975 s |
| Minimum response latency | 7.919 s |
| Maximum response latency | 34.495 s |
| P90 response latency | 34.495 s |
| P95 response latency | 34.495 s |
| Positive decisions | 6 |
| Negative decisions | 6 |
| Unparseable decisions | 0 |
| Positive decision rate | 50.0% |
| Alerts | 6 |
| Severity distribution | high: 5, low: 1, unknown: 6 |
| Action distribution | none: 7, stop_cart: 5 |
| LLM calls | 7 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | No | unknown | none | 34.495 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 2 | Yes | low | none | 27.972 s | Log context: The system reports multiple 'Obstacle stream is busy' warnings and a 'dense obstacle field' detection from the LiDAR object converter. However, all reported objects are at a distant proximity (nearest surface > 4m), and no stop request was logged by the collision detector. MOLA localization remains healthy with an ICP quality above the threshold. |
| 3 | Yes | high | stop_cart | 25.975 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 4 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 5 | Yes | high | stop_cart | 10.840 s | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | Yes | high | stop_cart | — | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 7.919 s | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 11 | No | unknown | none | 27.037 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 12 | No | unknown | none | 24.778 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |

#### Run: anomaly_20260727_183711 (trial 2)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T19:01:40.822783+00:00 |
| Finished (UTC) | 2026-09-02T19:07:39.003551+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 358.181 s |
| Decisions measured for latency | 7 |
| Mean response latency | 22.466 s |
| Median response latency | 24.864 s |
| Minimum response latency | 8.593 s |
| Maximum response latency | 32.764 s |
| P90 response latency | 32.764 s |
| P95 response latency | 32.764 s |
| Positive decisions | 7 |
| Negative decisions | 5 |
| Unparseable decisions | 0 |
| Positive decision rate | 58.3% |
| Alerts | 6 |
| Severity distribution | high: 6, low: 1, unknown: 5 |
| Action distribution | none: 6, stop_cart: 6 |
| LLM calls | 7 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | No | unknown | none | 32.764 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 2 | Yes | low | none | 28.288 s | Multiple warnings from the lidar_object_to_obstacle and collision_detector nodes indicate a "dense obstacle field" with 5 obstacles detected at a distance of approximately 4.0m to 5.7m. While these are flagged as warnings, they are categorized as 'distant' proximity and no stop request was logged; therefore, the severity remains low. |
| 3 | Yes | high | stop_cart | 24.317 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 4 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 5 | Yes | high | stop_cart | 11.744 s | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | Yes | high | stop_cart | — | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 8.593 s | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | Yes | high | stop_cart | — | Log context: The system repeatedly reports 'Collision avoidance braking active' at a distance of 1.00m with the brake signal set (last_arduino_brake=1/255). While the braking briefly clears, it re-engages multiple times in rapid succession, indicating an immediate obstacle threat or persistent collision risk requiring a stop. |
| 11 | No | unknown | none | 26.689 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 12 | No | unknown | none | 24.864 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |

#### Run: anomaly_20260727_183711 (trial 3)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T19:07:39.019323+00:00 |
| Finished (UTC) | 2026-09-02T19:13:36.874754+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 357.855 s |
| Decisions measured for latency | 7 |
| Mean response latency | 22.612 s |
| Median response latency | 24.893 s |
| Minimum response latency | 9.123 s |
| Maximum response latency | 32.152 s |
| P90 response latency | 32.152 s |
| P95 response latency | 32.152 s |
| Positive decisions | 6 |
| Negative decisions | 6 |
| Unparseable decisions | 0 |
| Positive decision rate | 50.0% |
| Alerts | 6 |
| Severity distribution | high: 5, low: 1, unknown: 6 |
| Action distribution | none: 7, stop_cart: 5 |
| LLM calls | 7 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | Yes | low | none | 32.152 s | MOLA localization health intermittently transitioned to an unhealthy state at t=1785178623.397 where ICP quality dropped to 0.000 (below the 0.2 threshold), but it recovered quickly to a healthy status with an ICP quality of 0.318 shortly after. |
| 2 | No | unknown | none | 28.272 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 3 | Yes | high | stop_cart | 24.893 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 4 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 5 | Yes | high | stop_cart | 12.329 s | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | Yes | high | stop_cart | — | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 9.123 s | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 11 | No | unknown | none | 26.726 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 12 | No | unknown | none | 24.787 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |

### local_text_cache_20

| Parameter | Value |
| --- | --- |
| Configuration hash | 6717447d1477fd2b5d5cb247f7aeba0604e97fa3cc9733eca16142e87f679cdc |
| API frequency | 10.000 s |
| Maximum cached messages | 20 |
| Maximum cache age | 30.000 s |
| Minimum trigger importance | warning |
| Model provider | gemma4 |
| Model | gemma4:12b-it-qat |
| Local model | Yes |
| Model token context | 3072 |
| Vision enabled | No |
| Image context enabled | No |
| Routine image frames | 1 |
| Maximum image frames | 2 |

#### Aggregate results

| Metric | Result |
| --- | --- |
| Decisions measured for latency | 15 |
| Mean response latency | 19.808 s |
| Median response latency | 16.810 s |
| Minimum response latency | 3.918 s |
| Maximum response latency | 35.039 s |
| P90 response latency | 35.038 s |
| P95 response latency | 35.039 s |
| Positive decisions | 15 |
| Negative decisions | 18 |
| Unparseable decisions | 0 |
| Positive decision rate | 45.5% |
| Alerts | 15 |
| Severity distribution | None |
| Action distribution | None |
| LLM calls | 15 |
| Pending LLM calls at timeout | 0 |

#### Repeatability across runs

| Metric | Runs | Mean | Std. dev. | Min | Max |
| --- | --- | --- | --- | --- | --- |
| Mean response latency (seconds) | 3 | 19.808 | 0.294 | 19.470 | 20.002 |
| Elapsed time (seconds) | 3 | 312.226 | 0.013 | 312.213 | 312.238 |
| Positive decisions | 3 | 5.000 | 0.000 | 5.000 | 5.000 |
| Negative decisions | 3 | 6.000 | 0.000 | 6.000 | 6.000 |
| Alerts | 3 | 5.000 | 0.000 | 5.000 | 5.000 |
| LLM calls | 3 | 5.000 | 0.000 | 5.000 | 5.000 |

**Bag-level trial consistency**

| Bag | Completed trials | Trials with positives | Positive outcome rate | Same outcome | Same positive count | Positive counts by trial |
| --- | --- | --- | --- | --- | --- | --- |
| anomaly_20260727_183711 | 3 | 3 | 100.0% | Yes | Yes | 5, 5, 5 |

#### Run: anomaly_20260727_183711 (trial 1)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T19:13:36.893780+00:00 |
| Finished (UTC) | 2026-09-02T19:18:49.106526+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 312.213 s |
| Decisions measured for latency | 5 |
| Mean response latency | 19.470 s |
| Median response latency | 15.738 s |
| Minimum response latency | 3.918 s |
| Maximum response latency | 35.039 s |
| P90 response latency | 35.039 s |
| P95 response latency | 35.039 s |
| Positive decisions | 5 |
| Negative decisions | 6 |
| Unparseable decisions | 0 |
| Positive decision rate | 45.5% |
| Alerts | 5 |
| Severity distribution | high: 5, unknown: 6 |
| Action distribution | LLM: 2, none: 4, stop_cart: 5 |
| LLM calls | 5 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | No | unknown | none | 34.502 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 2 | No | unknown | LLM | 35.039 s | LLM call Failed |
| 3 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 4 | Yes | high | stop_cart | 15.738 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 5 | Yes | high | stop_cart | — | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | No | unknown | LLM | — | LLM call Failed |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 8.153 s | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 11 | Yes | high | stop_cart | 3.918 s | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |

#### Warnings

- Artifact/LLM-decision counts differ; unmatched records were preserved (artifacts=4, decisions=5).

#### Run: anomaly_20260727_183711 (trial 2)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T19:18:49.120629+00:00 |
| Finished (UTC) | 2026-09-02T19:24:01.358856+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 312.238 s |
| Decisions measured for latency | 5 |
| Mean response latency | 19.952 s |
| Median response latency | 16.810 s |
| Minimum response latency | 5.376 s |
| Maximum response latency | 35.038 s |
| P90 response latency | 35.038 s |
| P95 response latency | 35.038 s |
| Positive decisions | 5 |
| Negative decisions | 6 |
| Unparseable decisions | 0 |
| Positive decision rate | 45.5% |
| Alerts | 5 |
| Severity distribution | high: 5, unknown: 6 |
| Action distribution | LLM: 2, none: 4, stop_cart: 5 |
| LLM calls | 5 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | No | unknown | none | 33.314 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 2 | No | unknown | LLM | 35.038 s | LLM call Failed |
| 3 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 4 | Yes | high | stop_cart | 16.810 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 5 | Yes | high | stop_cart | — | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | No | unknown | LLM | — | LLM call Failed |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 9.222 s | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 11 | Yes | high | stop_cart | 5.376 s | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |

#### Warnings

- Artifact/LLM-decision counts differ; unmatched records were preserved (artifacts=4, decisions=5).

#### Run: anomaly_20260727_183711 (trial 3)

| Item | Value |
| --- | --- |
| Status | completed |
| Started (UTC) | 2026-09-02T19:24:01.374271+00:00 |
| Finished (UTC) | 2026-09-02T19:29:13.601580+00:00 |
| Recorded bag duration | 308.426 s |

**Run metrics**

| Metric | Result |
| --- | --- |
| Elapsed time | 312.227 s |
| Decisions measured for latency | 5 |
| Mean response latency | 20.002 s |
| Median response latency | 16.836 s |
| Minimum response latency | 5.502 s |
| Maximum response latency | 35.019 s |
| P90 response latency | 35.019 s |
| P95 response latency | 35.019 s |
| Positive decisions | 5 |
| Negative decisions | 6 |
| Unparseable decisions | 0 |
| Positive decision rate | 45.5% |
| Alerts | 5 |
| Severity distribution | high: 5, unknown: 6 |
| Action distribution | LLM: 2, none: 4, stop_cart: 5 |
| LLM calls | 5 |
| Pending LLM calls at timeout | 0 |

**Final decisions**

| # | Anomaly | Severity | Action | Model latency | Summary |
| --- | --- | --- | --- | --- | --- |
| 1 | No | unknown | none | 33.404 s | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 2 | No | unknown | LLM | 35.019 s | LLM call Failed |
| 3 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 4 | Yes | high | stop_cart | 16.836 s | [t=1785178712.708611683 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.84m/s, speed_source=/estimate_twist, speed_measurement_age=0.09s, last_arduino_throttle=141/255, last_arduino_brake=0/255 |
| 5 | Yes | high | stop_cart | — | [t=1785178718.409221761 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.61m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 6 | No | unknown | LLM | — | LLM call Failed |
| 7 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 8 | Yes | high | stop_cart | 9.248 s | [t=1785178741.203116694 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.16m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 9 | Yes | high | stop_cart | — | [t=1785178752.605560919 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.29m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |
| 10 | No | unknown | none | — | Invalid/malformed LLM response (non-JSON). Falling back to safe default. |
| 11 | Yes | high | stop_cart | 5.502 s | [t=1785178765.908549770 frame=motor_endpoint_frame] node=motor_endpoint importance=ERROR type=TEXT msg=Received Motor Endpoint Info: Collision avoidance braking active: distance=1.00m, estimated_cart_speed=0.20m/s, speed_source=/estimate_twist, speed_measurement_age=0.08s, last_arduino_throttle=0/255, last_arduino_brake=1/255 |

#### Warnings

- Artifact/LLM-decision counts differ; unmatched records were preserved (artifacts=4, decisions=5).

---

Raw periodic messages, API artifacts, and process logs are omitted from this report.
