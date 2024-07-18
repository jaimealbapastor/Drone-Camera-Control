# Epipolar stereo calibration

## Epipolar constraint

$$
\begin{array}{c}
\left[ x_l \quad y_l \quad z_l \right]
\begin{bmatrix}
0 & -t_z & t_y \\
t_z & 0 & -t_x \\
-t_y & t_x & 0
\end{bmatrix}
\begin{bmatrix}
x_l \\
y_l \\
z_l
\end{bmatrix} = 0 \\
T_\times
\end{array}
$$

$T_\times$ is called the translation matrix

$t_{3 \times 1}$ : Position of Right Camera in Left Camera's Frame  
$R_{3 \times 3}$ : Orientation of Left Camera in Right Camera's Frame

$$
\begin{array}{l}
\boxed{\mathbf{x}_l = \mathbf{R} \mathbf{x}_r + \mathbf{t}} \\[20pt]
\begin{bmatrix}
x_l \\
y_l \\
z_l
\end{bmatrix} 
= 
\begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
\begin{bmatrix}
x_r \\
y_r \\
z_r
\end{bmatrix}
+
\begin{bmatrix}
t_x \\
t_y \\
t_z
\end{bmatrix}
\end{array}
$$

Substituting into the epipolar constrain gives:

$$
\begin{array}{c}
\left[ x_l \quad y_l \quad z_l \right]
\begin{bmatrix}
0 & -t_z & t_y \\
t_z & 0 & -t_x \\
-t_y & t_x & 0
\end{bmatrix}
\begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
\begin{bmatrix}
x_r \\
y_r \\
z_r
\end{bmatrix}
= 0 \\[10pt]
\end{array}
$$

$$
\begin{array}{c}
\left[ x_l \quad y_l \quad z_l \right]
\boxed{
\begin{bmatrix}
e_{11} & e_{12} & e_{13} \\
e_{21} & e_{22} & e_{23} \\
e_{31} & e_{32} & e_{33}
\end{bmatrix} }
\begin{bmatrix}
x_r \\
y_r \\
z_r
\end{bmatrix}

= 0 \\[20pt]
\text{Essential Matrix } \mathbf{E}
\end{array}
$$

And so we have:
$$ E = T_\times R $$

$$
\begin{array}{ccc}
\begin{bmatrix}
e_{11} & e_{12} & e_{13} \\
e_{21} & e_{22} & e_{23} \\
e_{31} & e_{32} & e_{33}
\end{bmatrix}
& = &
\begin{bmatrix}
0 & -t_z & t_y \\
t_z & 0 & -t_x \\
-t_y & t_x & 0
\end{bmatrix}
\begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
\end{array}
$$

Given that $T_x$ is a **Skew-Symmetric** matrix $(a_{ij} = -a_{ji})$ and $R$ is an **Orthonormal** matrix,  
it is possible to *"decouple"* $T_x$ and $R$ from their product using ***Singular Value Decomposition***.

## Compute $E$

Epipolar constraint :
$$
\boxed{X_l^T E X_r = 0} \\[20pt]

\left[ x_l \quad y_l \quad z_l \right]
\begin{bmatrix}
e_{11} & e_{12} & e_{13} \\
e_{21} & e_{22} & e_{23} \\
e_{31} & e_{32} & e_{33}
\end{bmatrix}
\begin{bmatrix}
x_r \\
y_r \\
z_r
\end{bmatrix}
= 0
$$

$X_l$ : 3D position in left camera coordinates  
$E$ : 3x3 Essential Matrix  
$X_r$ : 3D position in right camera coordinates

Rewriting in terms of image coordinates :

$$
\left[ u_l \quad v_l \quad 1 \right] \mathbf{K}_l^{-1 T}
\begin{bmatrix}
e_{11} & e_{12} & e_{13} \\
e_{21} & e_{22} & e_{23} \\
e_{31} & e_{32} & e_{33}
\end{bmatrix}
\mathbf{K}_r^{-1}
\begin{bmatrix}
u_r \\
v_r \\
1
\end{bmatrix}
= 0
$$

$$
\left[ u_l \quad v_l \quad 1 \right]
\boxed{
\begin{bmatrix}
f_{11} & f_{12} & f_{13} \\
f_{21} & f_{22} & f_{23} \\
f_{31} & f_{32} & f_{33}
\end{bmatrix}
}
\begin{bmatrix}
u_r \\
v_r \\
1
\end{bmatrix}
= 0 \\[5pt]
\text{Fundamental Matrix} \ F
$$

Once the fundamental matrix $F$ is found, it is simple to recover 

$$ 
\boxed{
\begin{array}{c}
E = K_l^T F K_r \\[5pt]
E = T_\times R
\end{array}
}
$$

## Calibration procedure

For each point $i$ seen in both images:
$$
\left[ u_l^{(i)} \quad v_l^{(i)} \quad 1 \right]
\begin{bmatrix}
f_{11} & f_{12} & f_{13} \\
f_{21} & f_{22} & f_{23} \\
f_{31} & f_{32} & f_{33}
\end{bmatrix}
\begin{bmatrix}
u_r^{(i)} \\
v_r^{(i)} \\
1
\end{bmatrix}
= 0
$$

We can rearrange the terms to form a linear system:

$$
\begin{bmatrix}
u_l^{(1)} u_r^{(1)} & u_l^{(1)} v_r^{(1)} & u_l^{(1)} & v_l^{(1)} u_r^{(1)} & v_l^{(1)} v_r^{(1)} & v_l^{(1)} & u_r^{(1)} & v_r^{(1)} & 1 \\
u_l^{(i)} u_r^{(i)} & u_l^{(i)} v_r^{(i)} & u_l^{(i)} & v_l^{(i)} u_r^{(i)} & v_l^{(i)} v_r^{(i)} & v_l^{(i)} & u_r^{(i)} & v_r^{(i)} & 1 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\
u_l^{(m)} u_r^{(m)} & u_l^{(m)} v_r^{(m)} & u_l^{(m)} & v_l^{(m)} u_r^{(m)} & v_l^{(m)} v_r^{(m)} & v_l^{(m)} & u_r^{(m)} & v_r^{(m)} & 1
\end{bmatrix}
\begin{bmatrix}
f_{11} \\
f_{21} \\
f_{31} \\
f_{12} \\
f_{22} \\
f_{32} \\
f_{13} \\
f_{23} \\
f_{33}
\end{bmatrix}
= 
\begin{bmatrix}
0 \\
0 \\
\vdots \\
0
\end{bmatrix}
$$

Fundamental matrix $F$ and $kF$ describe the same epipolar geometry.
$$
\left[ u_l \quad v_l \quad 1 \right]
\begin{bmatrix}
f_{11} & f_{12} & f_{13} \\
f_{21} & f_{22} & f_{23} \\
f_{31} & f_{32} & f_{33}
\end{bmatrix}
\begin{bmatrix}
u_r \\
v_r \\
1
\end{bmatrix}
= 0 =
\left[ u_l \quad v_l \quad 1 \right]
\begin{bmatrix}
k f_{11} & k f_{12} & k f_{13} \\
k f_{21} & k f_{22} & k f_{23} \\
k f_{31} & k f_{32} & k f_{33}
\end{bmatrix}
\begin{bmatrix}
u_r \\
v_r \\
1
\end{bmatrix}
$$

Find least square solution for $F$ *(constrained linear square square problem / eigenvalue problem)*
$$\min_{\mathbf{f}} \|A\mathbf{f}\|^2 \quad \text{such that} \quad \|\mathbf{f}\|^2 = 1$$

