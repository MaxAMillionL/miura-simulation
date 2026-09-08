"""Individual wheel-drive control for a four-facet Miura robot."""
__all__ = ["Simulation", "HardwareBridge", "MotorCommand"]

def __getattr__(name):
    if name in __all__:
        from . import control
        return getattr(control, name)
    raise AttributeError(name)
