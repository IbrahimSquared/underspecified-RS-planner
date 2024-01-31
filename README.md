# Under-specified Reeds-Shepp Planning

Consider the scenario where a car-like robot begins at a certain configuration $p_{0} = (x_{0}, y_{0}, \theta_{0})$ in an $N\times{}M$ grid. By solving the under-specified Reeds-Shepp problem, we endow the robot with the knowledge of the ending orientations $\theta_{f}^{N\times{}M}$ that produce the shortest paths to all grid cells. This allows the robot to move anywhere in the grid, always at a minimum-distance basis. Using the computed $\theta_{f}^{N\times{}M}$, we can compute a shortest-distance transform in the style of Euclidian distance transforms for the grid. The latter is symmetrical and rotationally invariant in its local frame for a fixed radius $r$, so it can be computed once and stored offline and it can be interpolated at any point. 

$\Omega$, therefore, may be defined as the final configuration angle that results in the shortest possible path length $\forall \theta_{f}$ in $[-\pi,\pi)$, given $(x_{f}, y_{f})$ and $p_{0}$.
Let $\mathcal{S}$ be the set of all possible paths for all $\theta_{f}$ in $[-\pi,\pi)$ given $p_{0}$ and $(x_{f}, y_{f})$. As such, $\Omega$ may be computed as:
$$ 
\Omega =  \underset{\theta_{f} \in [-\pi,\pi)}{\mathrm{argmin}} \int_{p_{0}}^{p_{f}} | P'(t) | dt \quad \forall P \in \mathcal{S}.
$$

In this code, we provide the solution for this problem that is based on the paper we submitted: (TBD).