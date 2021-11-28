# -*- coding: utf-8 -*-
"""
Created on Sun Nov 28 18:25:59 2021

@author: ision
"""

var speed0 = 100       # nominal speed
var speedGain = 2      # gain used with ground gradient
var obstThrL = 10      # low obstacle threshold to switch state 1->0
var obstThrH = 20      # high obstacle threshold to switch state 0->1
var obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)

var state = 0          # 0=gradient, 1=obstacle avoidance
var diffDelta          # difference between ground sensors
var obst[2]            # measurements from left and right prox sensors

timer.period[0] = 10   # 10ms sampling time

onevent timer0
  # acquisition from ground sensor for going toward the goal
  diffDelta = prox.ground.delta[1] - prox.ground.delta[0]
  # acquisition from the proximity sensors to detect obstacles
  obst = [prox.horizontal[0], prox.horizontal[4]]
  if state == 0 and (obst[0] > obstThrH or obst[1] > obstThrH) then
    # switch from goal tracking to obst avoidance if obstacle detected
    state = 1
  elseif state == 1 and obst[0] < obstThrL and obst[1] < obstThrL then
    # switch from obst avoidance to goal tracking if obstacle got unseen
    state = 0
  end
  if  state == 0 then
    # goal tracking: turn toward the goal
    motor.left.target = speed0 - speedGain * diffDelta
    motor.right.target = speed0 + speedGain * diffDelta
  else
    # obstacle avoidance: accelerate wheel near obstacle
    motor.left.target = speed0 + obstSpeedGain * (obst[0] / 100)
    motor.right.target = speed0 + obstSpeedGain * (obst[1] / 100)
  end