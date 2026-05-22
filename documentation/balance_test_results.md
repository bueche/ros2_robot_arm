# Balance Test Results
## Sections
## Balance Test results v1
### Summary

<p align="center">
  <img src="../images/test.result.summary.jpg" alt="summary " width="800">
</p> 

### Detailed Results by Date
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
