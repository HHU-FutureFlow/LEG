"""Independent SymPy/SciPy verification; does not access target hardware."""
from pathlib import Path
import json
import numpy as np
import sympy as s
from scipy.linalg import solve_continuous_are, eigvals

OUT = Path(__file__).resolve().parent
mp, m, length, lm, ell, gravity, radius, iw, mw, ip, im = (
    1.674, 4.326, .2933, .1067, .00494, 9.8, .06, .000695, .5, .0257, .0419
)
x, theta, phi, vx, vt, vp, ax, at, ap, tw, tp = s.symbols(
    'x theta phi vx vt vp ax at ap tw tp', real=True
)
q = s.Matrix([x, theta, phi])
v = s.Matrix([vx, vt, vp])
acc = s.Matrix([ax, at, ap])
leg = s.Matrix([x-length*s.sin(theta), length*s.cos(theta)])
body = s.Matrix([x-(length+lm)*s.sin(theta)-ell*s.sin(phi),
                 (length+lm)*s.cos(theta)+ell*s.cos(phi)])
def acceleration(position):
    velocity = position.jacobian(q)*v
    return velocity.jacobian(q)*v + velocity.jacobian(v)*acc
al, ab = acceleration(leg), acceleration(body)
nm, pm = m*ab[0], m*(ab[1]+gravity)
n, p = mp*al[0]+nm, mp*(al[1]+gravity)+pm
residual = s.Matrix([
    (mw+iw/radius**2)*ax+n-tw/radius,
    ip*at-(p*length+pm*lm)*s.sin(theta)
         -(n*length+nm*lm)*s.cos(theta)-tw-tp,
    im*ap+tp-nm*ell*s.cos(phi)-pm*ell*s.sin(phi),
])
zero = dict.fromkeys([*q,*v,*acc,tw,tp],0)
mass = np.array(residual.jacobian(acc).subs(zero), float)
stiffness = -np.array(residual.jacobian(q).subs(zero), float)
input_force = -np.array(residual.jacobian([tw,tp]).subs(zero), float)
assert np.allclose(mass,mass.T)
assert np.all(np.linalg.eigvalsh(mass)>0)
acc_a = np.linalg.solve(mass,stiffness)
acc_b = np.linalg.solve(mass,input_force)
A = np.zeros((6,6)); B = np.zeros((6,2))
A[0,1] = A[2,3] = A[4,5] = 1
A[np.ix_([3,1,5],[2,0,4])] = acc_a
B[[3,1,5]] = acc_b
Q = np.diag([150,1,50,1,200,1]); R = np.diag([8,.5])
P = solve_continuous_are(A,B,Q,R)
K = np.linalg.solve(R,B.T@P)
current = np.array([[22.8061,1.2335,7.0178,2.3379,1.952,.4453],
                    [-1.4104,-.0769,1.7325,.2979,-28.2688,-2.4956]])
old = current.copy(); old[1,:2] *= -1
co = np.column_stack([np.linalg.matrix_power(A,j)@B for j in range(6)])
assert np.linalg.matrix_rank(co)==6
care_residual = A.T@P+P@A-P@B@np.linalg.solve(R,B.T@P)+Q
assert np.max(np.abs(care_residual))<1e-7
assert np.max(eigvals(A-B@K).real)<0
assert np.max(eigvals(A-B@np.round(K,4)).real)<0
assert np.max(eigvals(A-B@current).real)>0

# Reproduce the original script's sequential substitutions at linear order.
# Each expression still contains the OLD acceleration symbols.
c = mp*length+m*(length+lm)
old_ex = (tw/radius-(mp+m)*ax-c*at+m*ell*ap)/(mw+iw/radius**2)
old_et = (c*gravity*theta-c*ax
          -(mp*length**2+m*(length+lm)**2)*at
          +m*(length+lm)*ell*ap-tw+tp)/ip
old_ep = (tp+m*ell*ax+m*(length+lm)*ell*at
          -m*ell**2*ap+m*gravity*ell*phi)/im
bad = s.Matrix([old_et,old_ex,old_ep]).subs(at,old_et).subs(ax,old_ex).subs(ap,old_ep)
badA = np.zeros((6,6)); badB = np.zeros((6,2))
badA[0,1] = badA[2,3] = badA[4,5] = 1
badA[np.ix_([1,3,5],[0,4])] = np.array(bad.jacobian([theta,phi]),float)
badB[[1,3,5]] = np.array(bad.jacobian([tw,tp]),float)
original_R = np.diag([1,.25])
badP = solve_continuous_are(badA,badB,Q,original_R)
badK = np.linalg.solve(original_R,badB.T@badP)
assert np.allclose(abs(badK),abs(current),atol=5.1e-5,rtol=0)

def poles(k):
    return [[float(z.real),float(z.imag)] for z in eigvals(A-B@k)]
report = dict(
    state_order=['theta','dtheta','x_wheel','dx_wheel','phi','dphi'],
    input_order=['Tw_forward','Tp_on_leg_forward_swing'],
    phi_convention='body COM behind hip is positive',
    A=A.tolist(),B=B.tolist(),Q=Q.tolist(),R=R.tolist(),K=K.tolist(),
    poles_corrected=poles(K),poles_corrected_4_decimals=poles(np.round(K,4)),
    poles_current=poles(current),poles_old_log=poles(old),
    original_script_sequential_substitution_K=badK.tolist(),
    controllability_rank=int(np.linalg.matrix_rank(co)),
    max_care_residual=float(np.max(abs(care_residual))),
    validation='SymPy nonlinear residual linearization and SciPy CARE; MATLAB not executed; no hardware writes',
)
(OUT/'verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
np.set_printoptions(precision=7,suppress=True,linewidth=130)
for name,item in [('A',A),('B',B),('K corrected',K),('K from original bug',badK)]:
    print(name); print(item)
for name,k in [('corrected',K),('current',current),('old log',old)]:
    print(name,'closed-loop poles:',eigvals(A-B@k))
print('PASS: mass symmetry/positivity, controllability, CARE residual, rounded-K stability, original coefficient reproduction')
