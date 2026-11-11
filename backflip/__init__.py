"""minipupper-backflip: keyframe backflips for Mini Pupper 4 - simulation, servo-model fitting, trajectory
optimization and real-robot analysis.  See README.md."""
from .keyframes import Preset
from .robot import ENSEMBLE, ENSEMBLE_FLOORS, FITTED, FLOORS, Floor, Robot, ServoModel
from .sim import FlipResult, score, simulate

__all__ = ["Preset", "Robot", "ServoModel", "Floor", "FLOORS", "FITTED", "ENSEMBLE", "ENSEMBLE_FLOORS",
           "FlipResult", "simulate", "score"]
__version__ = "1.1.0"
