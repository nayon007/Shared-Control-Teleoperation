"""Default parameters. Symbols follow the paper (Sec. 3, Alg. 1)."""

from dataclasses import dataclass, asdict


@dataclass
class Params:
    # Teleop mapping (block 3)
    v_max: float = 0.10          # [m/s] maximum commanded Cartesian speed
    stick_deadzone: float = 0.10  # [-] radial dead-zone on stick axes

    # Constraint estimator (block 7, Eq. 4)
    beta: float = 0.05           # [-] low-pass gain, T_f ~ dt/beta
    v_min: float = 0.01          # [m/s] speed gate
    window: int = 50             # [-] N, number of direction samples

    # Assistance (block 4, Eq. 6)
    k_min: float = 0.05          # [-] residual off-axis gain when confident
    c0: float = 0.60             # [-] confidence where assistance starts
    c1: float = 0.90             # [-] confidence where assistance is full
    perp_leak: float = 0.0       # [1/s] off-axis set-point relaxation (not in
                                 #       paper; 0 = paper baseline, see README)

    # Haptics (block 8, Eq. 8)
    F0: float = 2.0              # [N] rumble dead-zone on |F_perp|
    F1: float = 20.0             # [N] |F_perp| giving full rumble
    kappa_u: float = 0.5         # [-] weight of rejected command in rumble
    Fpar0: float = 2.0           # [N] trigger dead-zone on |F_par|
    Fpar1: float = 15.0          # [N] |F_par| giving full trigger resistance

    # Evaluation
    slip_force: float = 30.0     # [N] |F_perp| counted as handle loss

    def as_dict(self):
        return asdict(self)
