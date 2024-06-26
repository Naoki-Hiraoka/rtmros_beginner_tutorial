# COLLADA to Eus

## 1. Example

ここでは、[VRML to Collada](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/model_conversion_beginner_tutorial/VRML_to_Collada.md)で生成した`/tmp/SampleRobot.dae`を変換する例で説明する.

COLLADAモデルをEusモデルに変換するには、変換元となるCOLLADAモデルに加えて、設定ファイル(`yaml`)が必要である. [samplerobot.yaml](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/model_conversion_beginner_tutorial/sample_model_conversion/models/samplerobot.yaml)を用意してあるので、`/tmp/samplerobot.yaml`にコピーして配置しておくこと.
```bash
rosrun euscollada collada2eus -I /tmp/SampleRobot.dae -C /tmp/samplerobot.yaml -O /tmp/samplerobot.l
```

### 1.1 yaml file
```yaml
rleg:
  - RLEG_HIP_R      : rleg-crotch-r
  - RLEG_HIP_P      : rleg-crotch-p
  - RLEG_HIP_Y      : rleg-crotch-y
  - RLEG_KNEE       : rleg-knee-p
  - RLEG_ANKLE_P    : rleg-ankle-p
  - RLEG_ANKLE_R    : rleg-ankle-r
rarm:
  - RARM_SHOULDER_P : rarm-shoulder-p
  - RARM_SHOULDER_R : rarm-shoulder-r
  - RARM_SHOULDER_Y : rarm-shoulder-y
  - RARM_ELBOW      : rarm-elbow-p
  - RARM_WRIST_Y    : rarm-wrist-y
  - RARM_WRIST_P    : rarm-wrist-p
  - RARM_WRIST_R    : rarm-wrist-r
lleg:
  - LLEG_HIP_R      : lleg-crotch-r
  - LLEG_HIP_P      : lleg-crotch-p
  - LLEG_HIP_Y      : lleg-crotch-y
  - LLEG_KNEE       : lleg-knee-p
  - LLEG_ANKLE_P    : lleg-ankle-p
  - LLEG_ANKLE_R    : lleg-ankle-r
larm:
  - LARM_SHOULDER_P : larm-shoulder-p
  - LARM_SHOULDER_R : larm-shoulder-r
  - LARM_SHOULDER_Y : larm-shoulder-y
  - LARM_ELBOW      : larm-elbow-p
  - LARM_WRIST_Y    : larm-wrist-y
  - LARM_WRIST_P    : larm-wrist-p
  - LARM_WRIST_R    : larm-wrist-r
torso:
  - WAIST_P      : torso-waist-p
  - WAIST_R      : torso-waist-r
  - CHEST        : torso-waist-y

##
## end-coords
##
larm-end-coords:
  translate : [0, 0, -0.12]
  rotate    : [0, 1,  0, 90]
  parent    : LARM_LINK6
rarm-end-coords:
  translate : [0, 0, -0.12]
  rotate    : [0, 1,  0, 90]
  parent    : RARM_LINK6
lleg-end-coords:
  translate : [0.00, 0, -0.07]
rleg-end-coords:
  translate : [0.00, 0, -0.07]
head-end-coords:
  translate : [0.08, 0, 0.13]
  rotate    : [0, 1, 0, 90]

##
## reset-pose
##
angle-vector:
  reset-pose : [-0.004457, -21.6929, -0.01202, 47.6723, -25.93, 0.014025, # rleg
                17.8356, -9.13759, -6.61188, -36.456, 0.0, 0.0, 0.0, # rarm
                -0.004457, -21.6929, -0.01202, 47.6723, -25.93, 0.014025, # lleg
                17.8356, 9.13759, 6.61188, -36.456, 0.0, 0.0, 0.0, # larm
                0.0, 0.0, 0.0] # torso
  reset-manip-pose : [0.0, -20.0, 0.0, 47.0, -27.0, 0.0, # rleg
                     30.0, 0.0, 0.0, -100.0,  9.0, -6.5,  36.5, # rarm
                     0.0, -20.0, 0.0, 47.0, -27.0, 0.0, # lleg
                     30.0, 0.0, 0.0, -100.0, -9.0, -6.5, -36.5, # larm
                     0.0, 0.0, 0.0] # torso
```

設定ファイルには、eusの各limbごとに、Colladaの関節名とeusの関節名の対応関係と、end-coordsの位置を記述する. ここに記述されていない関節は、eusの`:angle-vector`に含まれない.

また、`angle-vector:`の箇所に、任意のangle-vectorを記述する. ここに記述したangle-vectorは、`(send *robot* :hogehoge-pose)`で呼び出すことができる

## 1.2 use eus model
```lisp
(load "/tmp/samplerobot.l")
(setq *robot* (samplerobot))
```