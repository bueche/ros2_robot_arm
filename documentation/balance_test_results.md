# Balance Test Results
## Sections
## Balance Test results v1
### Summary
The following notes outline the test results so far and provide links to the details for each. 

The main metric for success in the V1 of the balancing effort is a "centered event". This is measured as 2 consecutive seconds in which the ball is within 15% of the center (pos < 0.15 on the unit circle). This event can happen only once for each pose as the test moves on to the next pose once it has been achieved. The following graph below charts the progress over the tests. 

<p align="center">
  <img src="../images/test.result.summary.jpg" alt="summary " width="800">
</p> 

Another interesting metric to track is the percentage of ball coordinate samples that fall within the tight range (pos<15%) or the near range (pos<30%). This is a useful metric to know if one is making incremental progress even if the ball centered metric does not increase or stays the same. 

<p align="center">
  <img src="../images/ball_tight_and_near_progress.jpg" alt="summary " width="800">
</p> 

### Detailed Test Result Conventions
There are several reporting conventions covered the results.
1. Each report file is tagged by date and test-number-of-the-day. We use YYMMDD.tN, where YY=2 digit year, MM = month, DD = day, and N = the test number of that day.
2. The raw test results are stored in a file with a naming convention: `test.results.YYMMDD.tN.cv3.txt`. Inside this file are the concatenated log output for (1) the `pose_test.py` utility, (2) the `wrist_balance_controller.py` node, (3) the `ball_balance_node.py`, (4) the `ball_detector_oak.py` or the `ball_detector_nvidia.py` (whichever was used in the test), and (5) the output from `system_watchdog.py`. In addition, some summary notes as to what was the main change from this test vs. the previous one are also provided.
3. The summary analysis is stored in a file with the naming convention: `test.analysis.YYMMDD.tN.cv3.txt`. This is created by running the `test_analysis.py` on the raw test results. This summarizes the entire test and also summarizes and ranks how each pose did.
4. Some corr (or correction) analysis is stored in a file with the naming convention: `test.corr.YYMMDD.tN.cv3.txt`. This provides a detailed analysis on how each correction did for a particular pose. Useful to spot the details of servo issues and to tune the PID values.
5. A  video of the rviz2 output is stored in a file with the naming convention: `test.video.YYMMDD.tN.cv3.txt`. 
6. Power logging output is stored in a file named `power_log_YYYYMMDD_NNNNNN.csv` this is a csv log output of the voltage, current, and temperature for the servos.

More details on the actual metric definitions can be found [here](./balance_metrics.md).

### Detailed Results by Date
#### 05-29-2026
- Summary: fixed the jiggle hammering bug. basically dialed back the number of gjiggle corrections to match the standard corrections (1 Hz). The D-term is still not working as desired leading to lower scores.
  - t1: duplicated issue from previous day
  - t2: jiggle logic fix tested. success!
  - t3: repeat of this fix

- Best run: 9 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/1wSghBDd543bUKWQG9RnK3jbwVvqRSKE1)

#### 05-28-2026
- Summary: Continued with ball detection and d-term issues.
  - t1: The jiggle logic that was in the ball_detector_nvidia (used to move the test forward if the ball was lost 
   by the ai model) was ported to the ball_detector_oak. This had a bug and it started to hammer the the servos with corrections causing a big dip in the ball centered events.
  - t2: same parameters but ball_detector_oak also updated to have the same containment heuristic added that ball_detector_nvidia had. still did poorly because the jiggle hammering issue was still present.
  
- Best run: 6 out of 11

- Raw test results: [here](https://drive.google.com/drive/folders/10qBUlCMGZYeKDe79L3tgGdXgbY-VFf6b)
#### 05-27-2026
- Summary: working through issues with ball detection and the new D-term approach.
   - t1: Redo of the imu for the Dterm due to the bug in the other approach antiwindup_decay:=0.2  and kd=0.07
Camera not setup well. Lots of ball and cup missing.
   - t2: Camera not setup well. Lots of ball and cup missing....repositioned the cup but it still needs to be higher.
   - t3: repositioned the cup but it still needs to be higher.
   
- Best run: 9 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/15OMOpQnW5coZrSKTk-wGhg9X7tkWkngf)

#### 05-25-2026
- Summary: 
  - t1: same parameters as previous last run. testing pid logger v5. end result: not feeling good about imu velocity values. they are too static. and the theory is that this is due to sampling of the imu values.
  - t2: New approach for D-term: defined by the dwrist_flex_dt (joint trajectory approach) vs. the imu. now supporting d_term_source:=joints (and imu --> legacy)
  - t3: same as a above tweaked the kd parameters
  - t4: same as a above and introduce some additional logic to handle the ball quickly passing the center.  antiwindup_decay:=0.2  and kd=0.05 
- Best run: 10 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/137WMwzBV4Gz3MZBRj6hqDPUynfYLXZxf)
#### 05-23-2026
- Summary: Introduced a new approach for the D-term instead of using the IMU now using the joint state data to compute the velocity of the servos. The imu values were really static.

- Best run: 10 out of 11 poses balanced

- Raw test results: [here](https://drive.google.com/drive/folders/137WMwzBV4Gz3MZBRj6hqDPUynfYLXZxf)

#### 05-23-2026
- Summary: No parameter changes from the last run on the previous day. created a new logger that is logging the pid output so I can study the sensors and their impact on the pid.

- Best run: 10 out of 11 poses balanced

- Raw test results: [here](https://drive.google.com/drive/folders/1NK0KE4UHPHZIe5EuDD7gsrXKRxyBEQ2-)

#### 05-21-2026
- Summary: No parameter changes from the last run on the previous day. The all 11 poses reached balance state. The system watchdog was moved from the Nvidia Nano to the PI in order to get an accurate inter-system latency measurement. In addition, the oscillation metric has been modified to take into  account the distance of the overshoot. The previous days data has not been updated with the new metric.
- Best run: 11 out of 11 poses balanced

- Raw test results: [here](https://drive.google.com/drive/folders/1SAEAZFQ_NCvkkLrv5R3Z77Q2QiYPSyYZ)

#### 05-20-2026
- Summary: Two tests all with the same parameters as the previous day's best run. These were redo of the last runs on the previous day (using Oak-D camera). Chief change was moving the inferencing from the Nvidia Nano / torch back to the Oak-D camera. This was attempted to get around spikes in the latency due to transient issues on topic serialization between the humble-based container (for Nvidia/torch) and the rest of the system which used Jazzy containers. This experiment was a success as the system performed well and the errors and latency spikes disappeared.
- Best run: 11 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1u0LKf98-EEiZf1zsIMm8q5eSyl1Q2h1t)

#### 05-19-2026
- Summary: Tuning of the D-term: First change: slightly increasing the D-term's from beyond yesterday's best run. But chief change was moving the inferencing from the Nvidia Nano / torch back to the Oak-D camera. This was attempted to get around spikes in the latency due to transient issues on topic serialization between the humble-based container (for Nvidia/torch) and the rest of the system which used Jazzy containers. This experiment was a success as the system performed well and the errors and latency spikes disappeared.

  - t1 : -p kd_flex:=0.05 -p kd_roll:=0.05 
  - t2 : -p kd_flex:=0.07 -p kd_roll:=0.07
  - t3 : -p kd_flex:=0.07 -p kd_roll:=0.07
- Best run: 11 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/12SIIar26ajCId2KxenKoLBnx-08IAtp2)

#### 05-18-2026
- Summary: Tuning of the D-terms. Motivated by the larger step (max_step_rad) causing ball to overshoot. 
0.02 was too little. 0.10 caused too much dampening (especially for pose 5). and 0.05 seemed the best so far.
  - t3: -p kd_flex:=0.02 -p kd_roll:=0.02  
  - t4: -p kd_flex:=0.05 -p kd_roll:=0.05
  - t5: -p kd_flex:=0.10 -p kd_roll:=0.10
  - t6: -p kd_flex:=0.05 -p kd_roll:=0.05
- Best run: 11 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1FO7Z4PCcVphPn5sKU4akMhfAkKErRM3L)

#### 05-17-2026
- Summary: Disabled stall logic, to leverage I-term boost only. The stall logic was redundant. Increased the max_step_rad to 0.10 radians. This allows the servo to make bigger steps by default. This allowed pose 5 to work consistently. All 11 poses made it to the balance state.
- Best run: 11 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1U4U-UoIMLNvcXACQfkVrv6aOotJpbh8c)

#### 05-16-2026
- Summary: Continued trying to solve pose 5 challenges (servo stalling on the small incremental targets). Found a bug in how the I-term pipeline that I thought was working before ... now realized that the stall logic was redundant.
- Best run: 10 out of 11 poses balanced

- Raw test results: [here](https://drive.google.com/drive/folders/1z1cPCYoLNKp98mv9OenhTXR3bl00UrI5)

#### 05-15-2026
- Summary: Continued to try to solve pose 5 balance scenario. Tuning of logic to boost when stalled.
- Best run: 8 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1glJg0WX8Qt_W_OdkAh3VcJ9vlDBaO7M6)

#### 05-12-2026
- Summary: Derived some new logic to attempt to solve pose 5 challenge: servo stalling and unable to make small increments.
- Best run: 8 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1_dgta9bZiPBLN-hbbgFrYAv4Kcrw6cu0)

#### 05-11-2026
- Summary: 
- Best run: 8 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1mLVPdBu6hI8HkZgBizKYOD7f58EED1n5)

#### 05-10-2026
- Summary: big progress with testing and tuning. 
  - t1:  -p max_step_rad:=0.02
  - t2: first attempt at D-term (calibrated IMU) -p kd_flex:=0.02 -p kd_roll:=0.02
  - t3: 2nd attempt ... arm shook too much total fail
  - t4: -p correction_hz:=2.0 ... reduced correction rate from once every 200ms to once every 500 ms .. still shaking
  - t5: -p correction_hz:=1.0 <--- this was the big win!  to get rid of shakes

- Best run: 8 out of 11 poses balanced (t5)
- Raw test results: [here](https://drive.google.com/drive/folders/1Aw71SPTRNM2B8XkhZjr-hQcE9Nj0HguZ)

#### 05-09-2026
- Summary: Initial use of anaysis metrics. Only P-term is being used. cup version 3 is being used.
- Best run: 1 out of 11 poses balanced
- Raw test results: [here](https://drive.google.com/drive/folders/1wS2cby8JCwtBqDidsu8bunQUOJk2VjV8)

#### 05-08-2026
- Summary:
- Best Run: 0 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/185mo2L-VGnIdWO9HgOUCZFh2qUUFjhYf)

#### 05-07-2026
- Summary:
- Best Run: 0 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/1N0kzlyP1XtdEWTQaDBJjzfdu7e6nE75E)

#### 04-29-2026
- Summary:
- Best Run: 0 out of 11
- Raw test results: [here](https://drive.google.com/drive/folders/1ONip-Px7ZV83DV2gUOei6lk9gwigBbRP)
