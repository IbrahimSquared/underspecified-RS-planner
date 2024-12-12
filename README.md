# Under-specified Reeds-Shepp Planning

Consider the scenario where a car-like robot begins at a certain configuration $p_{0} = (x_{0}, y_{0}, \theta_{0})$ in an $N\times{}M$ grid.

By solving the under-specified Reeds-Shepp problem, we endow the robot with the knowledge of the ending orientations $\theta_{f}^{N\times{}M}$ that produce the shortest paths to all grid cells. This allows the robot to move anywhere in the grid, always at a minimum-distance basis.

Using the computed $\theta_{f}^{N\times{}M}$, we can compute a shortest-distance transform in the style of Euclidian distance transforms for the grid. The latter is symmetrical and rotationally invariant in its local frame for a fixed radius $r$, so it can be computed once and stored offline and it can be interpolated at any point.

Given an initial configuration $p_{0} = (x_{0}, y_{0}, \theta_{0})$ and a final position $(x_{f}, y_{f})$ with an unspecified final orientation $\theta_{f}$, the goal is to find the final orientation $\Omega$ such that the resulting Reeds-Shepp path from $p_{0}$ to the final configuration $p_{f} = (x_{f}, y_{f}, \Omega)$ is the shortest possible. Formally, $\Omega$ is defined as:

$$
\Omega =  \underset{\theta_{f} \in [-\pi,\pi)}{\mathrm{argmin}} L(p_{0}, (x_{f}, y_{f}, \theta_{f})).
$$

where $L(p_{0}, (x_{f}, y_{f}, \theta_{f}))$ represents the length of the Reeds-Shepp path from the initial configuration $p_{0}$ to the final configuration $(x_{f}, y_{f}, \theta_{f})$. The objective is to determine $\theta_{f}$ such that the path length is minimized.

In this code, we provide the solution for this problem that is based on the paper we submitted: (TBD).

# Sample solution

Solved for a $1000\times{}1000$ grid map with the start configuration $p_{0} = (500,500,\frac{\pi}{2})$ and with a radius $r = 400$. <br>

## $\Omega^{N\times{}M}$ solution

![alt text](https://github.com/IbrahimSquared/underspecified-RS-planner/blob/main/samples/omega_values_M_cb.png) <br>

## Corresponding computed path lengths

We computed path lengths for all pairs $(p_{0}, p_{f_{i,j}})$ where $(i,j) \in N\times{}M$ grid.
![alt text](https://github.com/IbrahimSquared/underspecified-RS-planner/blob/main/samples/distance_values_M_cb.png) <br>

The path lengths were computed using our proposed accelerated Reeds-Shepp planner that runs an order of magnitude faster than state-of-the-art.

## Sample $\Omega^{N\times{}M}$ image generated using SFML in C++

Generated for a randomly chosen $\theta_{0}$. <br>
![alt text](https://github.com/IbrahimSquared/underspecified-RS-planner/blob/main/omega_values.png) <br>

## How to Use (To be completed)

Set compiler path if needed, make sure SFML libraries (libsfml-dev) are installed, then: <br>
`sudo apt install cmake` <br>
`sudo apt install g++` <br>
`mkdir build && cd build` <br>
`cmake ..` <br>
`make`

To use this project, follow these simple steps:

1. Clone the repository:
