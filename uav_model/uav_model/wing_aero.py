# -*- coding: utf-8 -*-
"""
Trapezoidal wing aerodynamic model
----
"""


class TrapezoidalWingAero:
    """Analytical aerodynamic model of a trapezoidal wing.

    Args:
        root_chord: chord at wing root (m).
        tip_chord: chord at wing tip (m).
        span: aerodynamic wing span, from root to tip. Fuselage is not taken into account (m).
        sweep: wing sweep in (m).
    """

    def __init__(
        self, root_chord: float, tip_chord: float, span: float, sweep: float
    ) -> None:

        self.__root_chord = root_chord

        if tip_chord > root_chord:
            msg = f"tip chord must be smaller or equal to root chord"
            raise ValueError(msg)
        self.__tip_chord = tip_chord

        self.__span = span

        self.__eff = root_chord / tip_chord

        self.__surface = (root_chord + tip_chord) * span / 2

        self.__lambda = span ** 2 / self.__surface

        self.__mean_chord = (
            (2.0 / 3)
            * (root_chord ** 2 + root_chord * tip_chord + tip_chord ** 2)
            / (root_chord + tip_chord)
        )

        self.__mean_sweep = (
            (sweep / 3) * (root_chord + 2 * tip_chord) / (root_chord + tip_chord)
        )

        self.__x_ac_le = self.__mean_sweep + 0.25 * self.__mean_chord

    @property
    def mean_chord(self) -> str:
        """Return mean chord."""
        return self.__mean_chord

    @property
    def surface(self) -> str:
        """Return wing surface (m2)."""
        return self.__surface

    @property
    def mean_sweep(self) -> str:
        """Return mean sweep."""
        return self.__mean_sweep

    @property
    def x_ac_le(self) -> str:
        """Return distance along x between leading edge and aerodynamic center."""
        return self.__x_ac_le

    @property
    def mean_chord(self) -> str:
        """Return mean chord."""
        return self.__mean_chord

    @property
    def span(self) -> str:
        """Return span (m)."""
        return self.__span

    @property
    def eff(self) -> str:
        """Return the effilement."""
        return self.__eff

    @property
    def _lambda(self) -> str:
        """Return the allongement."""
        return self.__lambda

    def wingLoadingSI(self, mass):
        '''Compute the wing loading.

        Args:
            other: aircraft mass (kg).

        Returns:
            The wing loading (kg/m2).
        '''
        return mass / self.__wing_surface

    def wingLoadingGDm2(self, mass):
        '''Compute the wing loading.

        Args:
            other: aircraft mass (kg).

        Returns:
            The wing loading (g/dm2).
        '''
        return self.wingLoadingSI(mass) * 10
