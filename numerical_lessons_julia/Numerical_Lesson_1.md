@def title = "Numerical Lesson #1 - Julia"
# ~~~
<span style='color:green'>ME 417 Control of Mechanical Systems</span>~~~

[Back to Lessons](/numerical_lessons/index.html)

### Numerical Lesson #1

## PART I: SIMULATING A SYSTEM RESPONSE

In this part you will be simulating a general second-order dynamic system using different numerical methods, and you will also be exploring the effect of changing the system parameters on the response.

The general form of a second-order dynamic system represented by Figure 1, is given by the transfer function

$$\frac{X( s )}{U( s )} = \frac{\omega_{n}^{2}}{s^{2} + 2\zeta\omega_{n}s + \omega_{n}^{2}}$$ 

Where $\omega_{n}$ and $\zeta$ are given by the definitions:
$\omega_{n} = \sqrt{\frac{K}{M}}$, $2\zeta\omega_{n} = \frac{f_{v}}{M}$

~~~
<center><img src="/assets/numerical_lessons_matlab/Numerical_Lesson_1/media/MSD.svg" style="max-width:450px"></center>
~~~

**~~~
<center>Figure 1 - General Second-Order (Spring-Mass-Damper) System</center>~~~**

**~~~
<span style='color:green'>EXERCISE 1</span>~~~**

Given the following parameters: $M = 7\text{kg}$, $K = 2.8N\text{/}m$, $f_{v} = 1.5N - s\text{/}m$

Simulate the response of the system to a step input using the command *stepplot()* in Julia from the ControlSystems.jl package. Repeat the step simulation with $f_{v} = 5$, and $f_{v} = 20$. Plot the three responses on the same plot. Explain the observed difference between the three responses.

```julia
```
<!-- Exercise 1 -->
\input{julia}{ex1.jl}
\input{plaintext}{output/ex1.txt}
\fig{ex1_plot}

Using the command *stepplot()* is useful for analyzing a system's response to a simple and  constant input to a system, but what if the input varies? We can instead use the command *lsimplot()*.

Repeat the task above, but to an input force defined by the piecewise function below, using the command *lsimplot()* from ControlSystems.jl package, from $t = 0s$ to $t = 100s$, and plot the result in a new figure.

$f( t ) = \left\{ \begin{matrix}
0 & ,0 < t \leq 1.5s \\
3 & ,1.5s < t \leq 8s \\
\begin{matrix}
 - 3 \\
0 \\
\end{matrix} & \begin{matrix}
,8s < t \leq 14s \\
14s < t \\
\end{matrix} \\
\end{matrix} \right.\ $

> Hint: typing ?lsimplot in Julia's REPL will guide you on how to use the function.


<!-- Exercise 1 - 2  -->
\input{julia}{ex1_2.jl}
\fig{ex1_2_plot}

Now, what if we don't know the input function in advance, more specifically, what if the input is a function of the output of the system $f(t,y,y_{\text{sp}})$, as in the case of any feedback control system. In this case we must use a different method to simulate the response. Also note that the methods above require the system to be linear and time-invariant. The next methods can be used to simulate any dynamic system (nonlinear and/or time-variant) if the differential equations and initial conditions are defined.

Given the following differential equation

$$a\ddot{x}(t) + b\dot{x}(t) + cx(t) = u(t)$$

We can separate it into two equations that can then be used in MATLAB to simulate the response. Let's re-write the equation variables as $x_{1} = x(t),x_{2} = \dot{x}(t),x_{3} = \ddot{x}(t)$. If we define a vector 
$
\mathbf{x}=\begin{bmatrix}
x_{1} \\
x_{2} \\
\end{bmatrix}$

then 
$\dot{\mathbf{x}} = \begin{bmatrix}
{\dot{x}}_{1} \\
{\dot{x}}_{2} \\
\end{bmatrix}$

In other words:
$$
\dot{\mathbf{x}} = \begin{bmatrix} 
{\dot{x}}_{1} \\                                      
{\dot{x}}_{2} \\                                      
\end{bmatrix} = \begin{bmatrix}                       
x_{2} \\                                              
\dfrac{u( t ) - cx_{1} - bx_{2}}{a} \\      
\end{bmatrix}
$$                                  

If we start off with some given initial conditions for $x_{1},x_{2}$, we can numerically integrate $\dot{\mathbf{x}}$ at consecutive time-steps to obtain 

$\mathbf{x =}\begin{bmatrix}
x_{1} \\
x_{2} \\
\end{bmatrix} = \begin{bmatrix}
x(t) \\
\dot{x}(t) \\
\end{bmatrix}$

over a given time period $t \in \lbrack 0,t_{f}\rbrack$. In the following code snippet is an example on how to simulate the above equation in Julia using [OrdinaryDiffEq.jl](https://github.com/SciML/OrdinaryDiffEq.jl). which includes a number of methods for solving ordinary differential equations.

<!-- Snippet 1 -->
\input{julia}{snp1.jl}
\fig{snp1_plot}


**~~~
<span style='color:green'>EXERCISE 2</span>~~~**

Obtain the differential equation from the transfer function on the first page from equation (1), into the form shown in equation (2), then simulate the response to a *sinusoidal input* of unity magnitude and $f = 0.1Hz$, $u( t ) = sin(2\pi ft)$, using an ODE solver from OrdinaryDiffEq.jl package, **change the value of the damping coefficient** $f_{v}$ to $f_{v} = 0.1\dot{x}$. You now have a  nonlinear system. Simulate for $t = \left\lbrack 0,\ 100s \right\rbrack$ and plot *both* $x,\dot{x}$ on the figure. Note that the differential equation for the spring-mass-damper system above can also be expressed as: 

$M\frac{d^{2}x}{dt^{2}} + f_{v}\frac{\text{dx}}{\text{dt}} + Kx = Ku$

<!-- Exercise 2 -->
\input{julia}{ex2.jl}
\fig{ex2_plot}

Another method for solving a differential equation is using basic numerical integration with a fixed time-step. This is generally less efficient for higher order systems, but is more flexible in simulating dynamic systems, specifically, in simulating systems in real-time (simulating the dynamics of a car in a video-game is an example).
Moreover, this method is sufficient for simulating many mechanical dynamic systems. The code snippet below shows an example of basic forward integration to simulate a second-order system response; the same system from equation (*3*).

<!-- Snippet 2 -->
\input{julia}{snp2.jl}
\fig{snp2_plot}

**~~~
<span style='color:green'>EXERCISE 3</span>~~~**

Repeat the previous task using basic numerical integration this time, with the following time steps $\Delta t = 0.01$, and $\Delta t = 0.001$. Then plot the responses from the ode45() method and the basic integration method, as three subplots on the same figure. Do you notice a difference in the response between $\Delta t = 0.01s$ and $\Delta t = 0.001s$?

<!-- Exercise 3 -->
\input{julia}{ex3.jl}
\fig{ex3_plot}

## PART II: MODELING & SIMULATING REAL-WORLD SYSTEMS.

### **Model 1. Satellite Attitude**

We wish to model the 2-D attitude of a space satellite body. The 2-D satellite body can be modeled as a rigid-body mass with a moment of inertia $I$, rotating about its center of gravity $\text{C.G.}$ If we neglect the reaction moment from the satellite solar panels attached to the body, the remaining forces acting on the satellite are the thrust forces from the on-board gas thrusters in addition to disturbances that can be modeled as a moment $M_{D}$, as shown on Figure 2.

~~~
<center><img src="/assets/numerical_lessons_matlab/Numerical_Lesson_1/media/Satellite_Attitude_2D.svg" style="max-width:600px"></center>
~~~

**~~~
<center>Figure 2 - Space Satellite Body 2-D Attitude Model</center>~~~**

**~~~
<span style='color:green'>EXERCISE 4</span>~~~**

Given $d = 2m$, $I = 3.5kg \cdot m^{2}$

Derive the equation of motion for the satellite body shown, then derive the transfer function relating the resultant thrust force $F_{R} = F_{1} - F_{2}$ to the angular position of the body $\theta$, that is, find $G( s ) = \dfrac{\Theta( s )}{F_{R}( s )}$.
Neglect $M_{D}$.
Given the system equation of motion, simulate the response to the following inputs, assuming $M_{D}(t) = 0$. Simulate the response using basic numerical integration.

$F_1 = \begin{cases} 0  &  0 < t < 2s \\ 2 & 2s \leq t < 4s \\  0 & 4s \leq t \end{cases}$,  $F_2 = \begin{cases} 0 & 0 < t < 4s \\ 2 & 4s \leq t < 6s \\  0 & 6s \leq t \end{cases}$ 

<!-- Exercise 4 -->
\input{julia}{ex4.jl}
\fig{ex4_plot}

The thruster input function above is termed a bang-bang command. The idea is to apply an equal but opposite set of moments in order to roll the satellite body over a defined angle. The gas thrusters can only act in one direction, so a pair is needed to move-then-stop. This input is termed an open-loop input, meaning it doesn't consider the actual response in the system (it's blind toward the system's actual response).
In reality it's quite impossible to control a satellite body in this fashion, we will need to employ a feedback controller to provide just the right amount of thruster force to perform a rotation. Next we will see the effect of adding just a tiny amount of  disturbance noise to the model.

Repeat the simulation but this time assume that the disturbance moment is modeled as a zero-mean Gaussian noise with $\sigma = 0.005$. Hint: to generate this random noise value use: $\sigma*randn( 1 )$

<!-- Exercise 4 - 2 -->
\input{julia}{ex4_2.jl}
\fig{ex4_2_plot}


### **Model 2. Simple Pendulum**

A simple pendulum from a uniform slender rod is shown on Figure 3.
~~~
<center><img src="/assets/numerical_lessons_matlab/Numerical_Lesson_1/media/SimplePendulum.svg" style="max-width:300px"></center>
~~~
**~~~
<center>Figure 3 - Pendulum</center>~~~**

The equation of motion for the pendulum is given as

$$\ddot{\theta}( I + m_{r}l^{2} ) + b\dot{\theta} + m_{r}glsin\theta = M$$ 

Where $M$ is the torque applied at the pivot, $m_{r}$ is the mass of the rod, $I$ is the rod's moment of inertia about its center of mass, $l$ is the distance from the pivot to the center of mass of the rod, $b$ is the rotational damping coefficient at the hinge and $g$ is the gravity constant

Note that the above equations are nonlinear due to the presence of the trigonometric term $\text{sinθ}$

**~~~
<span style='color:green'>EXERCISE 5</span>~~~**

Given the following parameters:

Parameter  |    $m_{r}$     |    $b$             |  $l$
-----------|----------------|--------------------|-------------
Value      | $3\text{kg}$   |  $0.01 N \cdot s$  | $19\text{cm}$

Simulate the response of the above nonlinear system given the following input function and plot the following responses: $\theta,\dot{\theta}$. Simulate using basic numerical integration.
If the response is unstable (grows out of bounds) try to reduce the integration time step $\Delta t$

$M = \begin{cases} 5N \cdot m & 0 < t < 3s  \\ 0 & 3s \leq t \end{cases}$

<!-- Exercise 5 -->
\input{julia}{ex5.jl}
\fig{ex5_plot}

In order to design a controller for this system using the tools we introduce in this course; the system must be linearized. In this case we can linearize the system using the small angle approximation assumption. We can do this because we generally intend to keep (control) the inverted pendulum close to $\theta \approx 0^{o}$.

Linearize the above equations using the small angle approximation. That is, assume $sin\theta = \theta$.

Repeat task ‎1 using the linearized model and compare the nonlinear versus the nonlinear responses, in two subplots, one for each of $\theta,\dot{\theta}$. Briefly explain the observed differences. At what value of $\theta$ does the approximation become noticeably inaccurate?

<!-- Exercise 5 - 2 -->
\input{julia}{ex5_2.jl}
\fig{ex5_2_plot}

Note that when experimenting with a controller design via simulation, we design the controller using the linearized model if we wish, but it is important to simulate the response as accurate as possible. That is, simulate the nonlinear response.
_
Explain why it is better to simulate the nonlinear model and not the linearized model, when testing a controller?

