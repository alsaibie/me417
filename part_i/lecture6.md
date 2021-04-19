@def title = "Part I L6 - Poles Zeros and System Response"
@def hascode = true

# Part I L6 - Poles, Zeros and System Response
## Lecture Video


[Lecture Handout](/part_i/ME417_-_Controls_-_Part_I_Lecture_6_Poles_Zeros_and_System_Response.pdf)

## Objectives

- Introduce the concepts of Poles and Zeros of a Transfer Function
- Review the response of a First-Order System
- Introduce Second-Order Systems

### Reading: Nise 4.1-4.4

## Where we are
In this lecture, we will start to classify systems based on their response characteristics. 
So far, we introduced the transfer function as a way to represent a system model, now we will introduce a different way to represent a system. We will introduce a graphical way to represent a system. We will also introduce standard performance specifications, that will help us in analyzing the performance of systems.

### The Control System Design Process
~~~
<br>
<center><img src="/part_i/lecture6_media/The_Control_System_Design_Process_2to3.svg" style="max-width:785px"></center>~~~


## Poles, Zeros and System Response
The output response of a system is the sum of the forced response and natural response

$c_t(t) = c_{forced}(t)+c_{natural}(t)$

Given the equation of motion for a system, we can mathematically obtain its output response But there are qualitative ways of studying the output response of the system. One technique is to look at the poles and zeros of a system and their relationship to the output response. 

Poles $\times$ and zeros $o$ can be derived from a system’s transfer function.

### Poles of a transfer function - definition
Poles (denoted by the symbol $\times$) of a transfer function $G(s)$ can either be:
- The values of $s$ that cause the transfer function $G(s)$ to become infinite, or
- Any roots of the denominator of the transfer function $G(s)$ that are common to the numerator’s.

Example: The two poles of $G(s)=\dfrac{1}{s(s+4)}$, are $s=0,s=−4$

Example: The three poles of $G(s)=\dfrac{(s+3)}{(s^2+6s+18)(s+3)}$, are $s=−3, s=−3±3i$

Note that mathematically $G(s)=\dfrac{(s+3)}{(s^2+6s+18)(s+3)}=\dfrac{1}{(s^2+6s+18)}$ where the latter form has only two poles; however, $s=−3$ is still is a pole of the original system, it is only that the effect of this pole is cancelled in this case. 

### Zeros of a transfer function - definition
Zeros (denoted by the symbol $o$) of a transfer function $G(s)$ can either be
- The values of $s$ that cause the transfer function $G(s)$ to become zero, or
- Any roots of the numerator of the transfer function 𝐺(𝑠) that are common with the denominator’s
  
Example: The zero of the transfer function $G(s)=\dfrac{(3s+1)}{s}$, is $s=-\dfrac{1}{3}$

Example: The two zeros of the transfer function $G(s)=\dfrac{s(2s+1)}{(2s+1)}$, are $s=-\dfrac{1}{2}, s= 0$

In later sections, we will learn why its important to keep in mind the pole-zero cancellation behavior. 

## Poles, Zeros and the S-Plane

In the Laplace domain, we defined $ s=\sigma+\omega j$

The s-plane is where we plot the values of $s$. Poles and Zeros are graphically placed on the s-plane. The figure shows a graphical representation of the transfer function:

$G(s)=\dfrac{5s}{(s+5)}$

Note that the gain 5 is not captured on the graph (we will deal with expressing the gain value under the Root-Locus section)

~~~
<br>
<center><img src="/part_i/lecture6_media/pzexample.svg" style="max-width:485px"></center>~~~

## Pole Location and Time Response
With respect to the pole location on the s-plane:

- A pole on the real negative axis produces an exponentially decaying response. 
    - $G(s)=\dfrac{1}{(s+3)}$
- A pole pair on the imaginary axis produce a sinusoidal response.
    - $G(s)=\dfrac{1}{s^2+18}$
- A pole on the real positive axis produces an exponentially growing response. (unstable response)
    - $G(s)=\dfrac{1}{(s-4)}$
- A pole at the origin produces a step response.
    - $G(s)=\dfrac{1}{s}$

## Poles and Zeros and Response Function to a Step Input
~~~
<br>
<center><img src="/part_i/lecture6_media/Figure4_1.svg" style="max-width:785px"></center>~~~

## Example 1

*For the system with the transfer function $G(s)=\dfrac{C(s)}{R(s)}=20\dfrac{(s+4)(s+3)(s-8)}{(s+1)(s+2)^2}$, write, by inspection, the output $c(t)$ in general terms, to a ramp input $R(s)=\dfrac{1}{s^2}$ and specify the natural and forced response parts.*

## First-Order Systems
A first-order system is a system whose highest derivative order is 1, or who’s characteristic equation is of degree 1.
- Example: The system (internal system dynamics) with $G(s)=\dfrac{(s+2)}{(s+3)(s+3)}=\dfrac{1}{(s+3)}$, is a not first order system per se, but the system response is a first-order response, due to the pole-zero cancellation. 
- Example: The system with $G(s)=10\dfrac{(s+3)}{(s+5)}$ is a first-order system with a zero

Examples of First Order Systems:
- Heat Transfer (Thermometer)
- Interest Rate Growth
- RC Circuit

## System Performance Specifications

In the context of control system design, there are well defined performance specifications that we evaluate, such as:
Rise Time $T_r$, Settling Time $T_s$, Time Constant $\tau$, Percentage Overshoot, Peak Time $T_p$. These are common characteristics, but performance specifications are not limited to them in the design of real control systems. We will define some performance specifications for First-Order systems.

Performance specifications for first-order systems are defined for $G(s)=\dfrac{a}{(s+a)}$, a first-order system with no zero. 

Moreover, the specifications are defined for a response to a unit-step input.

- $R(s)=\dfrac{1}{s}$
- $C(s)=\dfrac{a}{s(s+a)}$
- $c(t)=c_{forced}(t)+c_{natural}(t)=1-e^{-at}$

### **Time Constant**
$\tau$: $\tau=\dfrac{1}{a}$, $c(t=\dfrac{1}{a})=1-e^{-1}=0.63$

Time constant is the time required for $e^{-at} $to decay to $37\%$ of its initial value. The reciprocal of the time constant, is called the exponential frequency

### **Rise Time $T_r$**
The time for the response to go from 0.1 to 0.9 of its final value

- For the $1^{st}$ Order System $G(s)=\dfrac{a}{(s+a)}$, $T_r=\dfrac{2.2}{a}$

### **Settling Time $T_s$**
The time required for the response to reach and stay within $2\%$ of its final value.

For the 1st Order System $G(s)=\dfrac{a}{(s+a)}$, $T_s=\dfrac{4}{a}$

Other times, the settling time is defined with a different than $2\%$ target, $5\%$ is common as well. In this course we will use the $2\%$ target when defining $T_s$

Remember that the above defined equations are strictly for a system with the transfer function of the form $G(s)=\dfrac{a}{(s+a)}$, in response to a unit step input.
~~~
<br>
<center><img src="/part_i/lecture6_media/Figure4_5.svg" style="max-width:385px"></center>~~~

## Example 2
*The figure shows the response of a system measured experimentally, identify the type of system response and find the time constant, rise time and settling time of the system.*

~~~
<br>
<center><img src="/part_i/lecture6_media/FirstOrderResponse.svg" style="max-width:485px"></center>~~~

## Second-Order Systems