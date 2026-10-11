\# Python–C++ Rigid-Body Derivative Validation



\## Purpose



This procedure compares the instantaneous rigid-body derivatives produced by the Python reference implementation and the C++ implementation.



It does not validate the full integrated trajectory or the physical accuracy of the vehicle parameters.



\## State and coordinate conventions



\- Inertial frame: North-East-Down (NED).

\- Body frame: Forward-Right-Down (FRD).

\- Quaternion order: scalar-first `\[qw, qx, qy, qz]`.

\- Quaternion multiplication: Hamilton product.

\- Position: m.

\- Velocity: m/s.

\- Angular velocity: rad/s.

\- Force: N.

\- Torque: N·m.

\- Translational acceleration: m/s².

\- Angular acceleration: rad/s².



\## Model equations



Position kinematics:



\\\[

\\dot{\\mathbf r}^{NED}=R\_{NB}\\mathbf v^B

\\]



Body-frame translational dynamics:



\\\[

\\dot{\\mathbf v}^B =

\\frac{\\mathbf F^B}{m}

+R\_{NB}^{T}\\mathbf g^{NED}

\-\\boldsymbol\\omega^B\\times\\mathbf v^B

\\]



Rotational dynamics for diagonal inertia:



\\\[

\\dot{\\boldsymbol\\omega}^{B}

=I^{-1}

\\left(\\boldsymbol\\tau^B

\-\\boldsymbol\\omega^B\\times(I\\boldsymbol\\omega^B)\\right)

\\]



Quaternion kinematics:



\\\[

\\dot q\_{NB}=\\frac{1}{2}q\_{NB}\\otimes

\\begin{bmatrix}0\\\\p\\\\q\\\\r\\end{bmatrix}

\\]



The quaternion is normalized before calculating the rotation matrix and quaternion derivative, matching the current implementation.



\## Comparison procedure



The Python script passes the same state, body force, and body torque to the C++ executable. Both implementations use the same rigid-body parameters.



The 13 derivative components are compared in this order:



1\. Position derivative (3 values).

2\. Body velocity derivative (3 values).

3\. Quaternion derivative (4 values).

4\. Angular velocity derivative (3 values).



\## Numerical acceptance criterion



The comparison uses:



\- Absolute tolerance: \\(10^{-10}\\).

\- Relative tolerance: \\(10^{-10}\\).



A case passes when all 13 components satisfy the NumPy `allclose` criterion with these tolerances.



\## Test cases



\- `level\_static`: level attitude with thrust balancing gravity.

\- `translated\_rotating`: nonzero body velocity, attitude, angular rates, force, and torque.

\- `coupled\_rotation`: non-identity quaternion and nonzero angular rates to exercise rotational coupling.



\## Limitations



Passing these tests establishes numerical agreement for the tested instantaneous derivatives only.



It does not establish:

\- Full 17-state trajectory equivalence.

\- Numerical integration equivalence.

\- Motor dynamics equivalence.

\- Experimental validation of thrust or torque coefficients.

\- Real-world flight safety or vehicle performance.



