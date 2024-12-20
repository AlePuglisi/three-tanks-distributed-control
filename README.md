# three-tanks-distributed-control
MATLAB | YALMIP | Centralized,Decentralized,Distributed Control | Linear Matrix Inequality (LMI) | Control Theory

This repository contains the MATLAB code and Report of the final group project of a course on [Networked Control systems](https://www11.ceda.polimi.it/schedaincarico/schedaincarico/controller/scheda_pubblica/SchedaPublic.do?&evn_default=evento&c_classe=810213&__pj0=0&__pj1=7f724b397a9ae97bd6cf089503f5774c).


## Introduction 
In this project, we solve a benchmark problem in control theory, the (linearized) modeling and control of a system of three tanks in series.<br/>
In particular, coherently with the course topics, we implement different control schemes (centralized, decentralized, distributed).<br/>
Simulation and performance analysis are done both in continuous-time and discrete-time domains.<br/>

<image width=390 height=280 src=https://github.com/user-attachments/assets/ed309489-9679-4e45-af91-1218c316ddf7>
<image width=420 height=280 src=https://github.com/user-attachments/assets/4a309df1-49a2-4c43-bc94-6c02ca16e1cc>
  
<br/>
<br/>
The project is implemented in **MATLAB**, and **YALMIP** (with sedumi or SDTP3 for low-level optimization) as the optimization toolbox for LMI.<br/>
Controller tuning relies on Linear Matrix Inequality (LMI) theory (simple but efficient). 

<br/>
<br/>

If you want more details on the modeling, system decomposition, and control tuning, refer to our [presentation](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/Presentation_project.pdf)

## Objective

Given the continuous time state-space description of the system:
- Decompose the model
- Perform open-loop stability analysis
- Define different state-feedback control structures (centralized, decentralized, distributed)
- Analyze each structure and compute the control gains to achieve satisfactory closed-loop performances.
- Plot the simulation results and compare the performance of the control structures. 

For a detailed task description, refer to the general [Project work assignment](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/general_project.pdf) and our [Three tanks assignment](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/Three_tanks_project.pdf)

## MATLAB Code 

> [!IMPORTANT]
> To run the code, you need [YALMIP](https://yalmip.github.io/) toolbox installed, and a low-level Linear Programming solver like [sedumi](https://github.com/sqlp/sedumi) or [SDTP3](https://github.com/sqlp/sdpt3).<br/>
> If you have some problem installing it, you can refer to this [presentation](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/YALMIP_HowtoInstall.pdf) provided by my professor. 

MATLAB scripts and functions are organized as follows: 
- **Functions**:<br/>
  Used in the main project scripts for fixed mode analysis and control gain tuning.<br/>
  For details, look at the function description (the control structure is specified as input)
    - ``di_fixed_modes.m``: compute system fixed mode<br/>
    
  The functions below are based on LMI resolution with YALMIP:
    - ``LMI_CT_DeDicont.m`` and ``LMI_DT_DeDicont.m``: Continuous (CT) and Discrete(DT) pole-placement control tuning
    - ``LMI_CT_H2.m`` and ``LMI_DT_H2.m``: Continuos (CT) and Discrete(DT) H2 optimal control tuning 
    - ``LMI_CT_opt1.m`` and ``LMI_DT_opt1.m``: Continuos (CT) and Discrete(DT) pole-placement plus control action rate limitation tuning.<br/>
        (This is the LMI I'm proud of, being my own creation, look at this part of the [presentation](https://github.com/AlePuglisi/three-tanks-distributed-control/blob/main/Report/Control_rate_limitation.pdf)) 
      
- **Scripts**:<br/>
  Even if it seems long and complex, don't panic! Most of it is related to plot signals and zero/poles analysis.<br/>
  In all the scripts below, we start with the system modeling, decomposition, control structure definition, and fixed mode computation.<br/>
  Then, an open-loop analysis and closed-loop control for different control structures.<br/>
  Each script differs by the controller gain tuning technique, based on the previously explained functions.  
   - ``PROJECT.m``: Tuning based on pole-placement 
   - ``PROJECT_H2.m``: Tuning based on H2
   - ``PROJECT_EXT.m``: Tuning based on the "extended system" technique, by pole-placement plus control action rate limitation.  


To run the code, open the PROJECT script in Matlab and run it. <br/>
Many images will be opened with the simulation results. 

## Conclusion and Comments

This MATLAB code and the project report can be helpful for Automation and Control Students. <br/>
Providing a simple way to define and solve LMIs, and a powerful tool to tune controllers for even bigger MIMO (Multi Input Multi Output) systems.<br/>

Experiment with it, and feel free to open an issue, reach out to me for any questions!

