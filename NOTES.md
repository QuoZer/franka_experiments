- [**State-of-the-art**](#state-of-the-art)
  - [First meeting references](#first-meeting-references)
    - [**AIC - A Novel Adaptive Controller for Robot Manipulators Based on Active Inference**](#aic---a-novel-adaptive-controller-for-robot-manipulators-based-on-active-inference)
    - [**Reinforcement learning of motor skills using Policy Search and human corrective advice**](#reinforcement-learning-of-motor-skills-using-policy-search-and-human-corrective-advice)
  - [Further research](#further-research)
    - [**Tactile Guidance for Policy Adaptation**](#tactile-guidance-for-policy-adaptation)
    - [**Learning Human Objectives from Sequences of Physical Corrections**](#learning-human-objectives-from-sequences-of-physical-corrections)
    - [**Physical Interaction as Communication - Learning Robot Objectives Online from Human Corrections**](#physical-interaction-as-communication---learning-robot-objectives-online-from-human-corrections)
    - [**Trajectory Deformations from Physical Human-Robot Interaction**](#trajectory-deformations-from-physical-human-robot-interaction)
    - [**A Review of Intent Detection, Arbitration, and Communication Aspects of Shared Control for Physical Human–Robot Interaction**](#a-review-of-intent-detection-arbitration-and-communication-aspects-of-shared-control-for-physical-humanrobot-interaction)




# **State-of-the-art**

## First meeting references
Starting papers for the project. 
Keywords: human intervention, human feedback loop, human adaptive learning for robots, phri

### **AIC - A Novel Adaptive Controller for Robot Manipulators Based on Active Inference**
[Paper link](https://ieeexplore-ieee-org.libproxy.aalto.fi/document/9000729/media#media) | 
[Lib repo](https://github.com/cpezzato/active_inference)   | 
[Sim repo]( https://github.com/cpezzato/panda_simulation)

**Summary**

In this paper, they present a novel adaptive controller for robot manipulators, inspired by a recent theory of the brain, which does not require accurate plant dynamics, and that is less sensitive to large parameters variation.

The proposed control scheme is based on the general free-energy principle proposed by Karl Friston, and redefined in engineering terms. The performance of the novel AIC has been compared with that of a state-of-the-art MRAC, in different pick and place scenarios. The controllers have been tuned using a considerably inaccurate model of the robot arm on purpose. The links have been approximated as cuboids, and 20% random uncertainty in each link's mass has been assumed.

The AIC and MRAC have been tested against large external disturbances such as a human pushing the robot during motion. AIC resulted more compliant than MRAC, showing at the same time a faster and less oscillatory disturbance rejection. The AIC shows better adaptability properties, allowing to transfer from simulation to real setup without re-tuning. In addition, the AIC resulted easier to tune and implement.


### **Reinforcement learning of motor skills using Policy Search and human corrective advice**
[Paper link](https://journals-sagepub-com.libproxy.aalto.fi/doi/pdf/10.1177/0278364919871998)

**Summary**

In this paper, authors use COACH (Celemin and Ruiz-del Solar, 2018) as the mechanism for interactive corrections as it allows human feedback to be introduced during robot execution.

The trainer has to advise the correction immediately after the execution of the action to be modified. The binary signals increase or decrease the magnitude of the executed action, and this signal can be given independently for every degree of freedom that composes the action vector. 

Relevant reference:  Argall et al. (2011) proposed a system that allows the modification of a primitive during execution with tactile feedback. If the user provides a correction with an effector displacement with respect to the original trajectory, the displacement is **applied from there** on to the rest of the path, then all the data points are recorded to rederive the policy after the execution. => *Tactile Guidance for Policy Adaptation*

## Further research

### **Tactile Guidance for Policy Adaptation** 
[Paper link]()

**Summary**

The human teacher may choose to offer a tactile correction at any timestep of the execution. If detected, the robot learner translates the tactile feedback into an incremental shift in robot pose, according to mapping M. The robot controller is then passed the new adjusted pose, for which the incremental shift is added to the current robot pose. The influence of this incremental shift is maintained over multiple timesteps, through an offset parameter δ that maintains a sum of all adjustments seen during the execution and is added to the pose prediction at each execution timestep.

Method relies on pressure sensors around the wrist of a robot.


### **Learning Human Objectives from Sequences of Physical Corrections**
[Paper link](https://arxiv.org/pdf/1907.03976.pdfhttps://vtechworks.lib.vt.edu/bitstream/handle/10919/108318/li_icra2021.pdf;jsessionid=5573746FD70F44F107B1012A3964DE28?sequence=2)

**Summary**

Human Corrections. The robot learns about the human’s reward — i.e., the true reward weights — from physical corrections. Intuitively, these corrections are applied forces and torques which push, twist, and guide the robots, deforming its trajectory. Considers all correction interconnected. 

*No code*


### **Physical Interaction as Communication - Learning Robot Objectives Online from Human Corrections**
[Paper link](https://vtechworks.lib.vt.edu/bitstream/handle/10919/108316/losey_ijrr2021.pdf?sequence=2)

**Summary**

Within the proposed framework human interactions become observations about the true objective. They introduce approximations to learn from and respond to pHRI in real-time. Alters the trajectory according to correction with trajectory deformations and learns the new weights to replan a new trajectory closer to the human's objective. 

*No code*


### **Trajectory Deformations from Physical Human-Robot Interaction**
[Paper link](https://arxiv.org/pdf/1710.09871.pdf)

**Summary**

Describes how to deform a trajectory online based on human interactions. Compared to *Physical Interaction as Communication* paper doesn't try to learn a human-desired trajectory. Instead an optimal deformation variance is computed and then applied to the trajectory (of the ned effector or each joint separately) proportionally to the external force and the scaling factor. 
Implemented in My repo as ShyController.


### **A Review of Intent Detection, Arbitration, and Communication Aspects of Shared Control for Physical Human–Robot Interaction**
[Paper link](https://asmedigitalcollection.asme.org/appliedmechanicsreviews/article/70/1/010804/443697/A-Review-of-Intent-Detection-Arbitration-and)

**Summary**

A review of the state-of-the-art methods of shared control for physical human-robot interaction. The paper is focusing on three main sections: intent detection, arbitration, and communication. 
