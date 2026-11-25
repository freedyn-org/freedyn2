"""Modal data of a flexible body in the floating frame of reference.

Body frame: the reference frame of the FE model (positions s_p of the nodes). Modal
coordinates eta (nm), nodal displacement u_p = Phi_p eta, small nodal rotation psi_p = Psi_p eta.
The inertia invariants follow Adams (Ottarsson, "Adams/Flex theory"), with the integrals as sums
over the nodes p (mass m_p, rotational inertia J_p, nodal rotational inertia only where Adams
keeps it):

    I1 = sum m_p                          mass
    I2 = sum m_p s_p                      (3)       first moment
    I3 = sum m_p Phi_p                    (3 x nm)
    I4 = sum m_p s~_p Phi_p + J_p Psi_p   (3 x nm)
    I5_j = sum m_p phi~_pj Phi_p          (nm x 3 x nm)
    I6 = sum m_p Phi_p^T Phi_p + ...      (nm x nm) generalized mass
    I7 = sum m_p s~_p^T s~_p + J_p        (3 x 3)   inertia tensor of the undeformed body
    I8_j = sum m_p s~_p phi~_pj           (nm x 3 x 3)
    I9_jk = sum m_p phi~_pj phi~_pk       (nm x nm x 3 x 3)

so that with u = s + Phi eta (body frame) the kinetic energy of the body with velocities
v = [Rdot (global); omega_bar (body frame); etadot] is

    T = 1/2 m Rdot.Rdot + Rdot.A (-S~ omega_bar + I3 etadot) + 1/2 omega_bar.J omega_bar
        + omega_bar.(I4 + sum_j eta_j I5_j) etadot + 1/2 etadot.I6 etadot,
    S = I2 + I3 eta,
    J = I7 - sum_j eta_j (I8_j + I8_j^T) - sum_jk eta_j eta_k I9_jk.

Invariants an MTX file does not contain (I5, I9 for partial coupling) are zero.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ModalData:
    mass: float
    I2: np.ndarray                 # (3,)
    I3: np.ndarray                 # (3, nm)
    I4: np.ndarray                 # (3, nm)
    I5: np.ndarray                 # (nm, 3, nm): I5[j] = I5_j
    I6: np.ndarray                 # (nm, nm)
    I7: np.ndarray                 # (3, 3)
    I8: np.ndarray                 # (nm, 3, 3)
    I9: np.ndarray                 # (nm, nm, 3, 3)
    K: np.ndarray                  # (nm, nm) generalized stiffness
    frequencies: np.ndarray        # (nm,) Hz
    node_labels: np.ndarray        # (nn,) int
    node_positions: np.ndarray     # (nn, 3) body frame
    Phi: np.ndarray                # (nn, 3, nm) translational mode shapes at the nodes
    Psi: np.ndarray                # (nn, 3, nm) rotational mode shapes at the nodes
    has_I5: bool = True            # full coupling data present (False: partial coupling MTX)
    has_I9: bool = True
    units: tuple = ()
    source: str = ""
    removed_modes: list = field(default_factory=list)

    @property
    def nm(self):
        return self.K.shape[0]

    def node_index(self, label):
        idx = np.nonzero(self.node_labels == int(label))[0]
        if not len(idx):
            raise KeyError(f"node {label} is not in the modal data")
        return int(idx[0])

    def damping(self, zeta):
        """Modal damping matrix D = 2 zeta_i omega_i (mass normalized modes, as the legacy
        FeLib_MtxConverter::computeDred); zeta: number or list (the last value for the rest)."""
        z = np.atleast_1d(np.asarray(zeta, float))
        z = np.concatenate([z, np.full(max(0, self.nm - len(z)), z[-1])])[: self.nm]
        omega = 2 * np.pi * self.frequencies
        return np.diag(2 * z * omega * np.diag(self.I6))

    def select(self, keep):
        """Modal data with only the modes keep (indices), with the viewer and stress data of a
        .fdflex (viz, mesh, spiders, meta) selected too."""
        k = np.asarray(keep, int)
        out = self._select(k)
        viz = getattr(self, "viz", None)
        out.viz = None if viz is None else (viz[0], viz[1], viz[2][:, :, k])
        mesh = getattr(self, "mesh", None)
        out.mesh = None if mesh is None else (mesh[0], mesh[1], mesh[2], mesh[3][:, :, k])
        out.spiders = getattr(self, "spiders", [])
        out.meta = dict(getattr(self, "meta", None) or {})
        return out

    def _select(self, k):
        return ModalData(
            mass=self.mass, I2=self.I2, I3=self.I3[:, k], I4=self.I4[:, k], I5=self.I5[np.ix_(k, range(3), k)],
            I6=self.I6[np.ix_(k, k)], I7=self.I7, I8=self.I8[k], I9=self.I9[np.ix_(k, k)], K=self.K[np.ix_(k, k)],
            frequencies=self.frequencies[k], node_labels=self.node_labels, node_positions=self.node_positions,
            Phi=self.Phi[:, :, k], Psi=self.Psi[:, :, k], has_I5=self.has_I5, has_I9=self.has_I9, units=self.units,
            source=self.source, removed_modes=self.removed_modes)

    def center_of_mass(self):
        return self.I2 / self.mass
