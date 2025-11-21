from WBR_class import WBR
import WBR_balance_controller
import WBR_state_estimator
import WBR_velocity_controller
import inspect

# Helper to attach all top-level functions from a module to WBR
def attach_module_functions(cls, module):
    for name, func in inspect.getmembers(module, inspect.isfunction):
        setattr(cls, name, func)

# Attach functions
attach_module_functions(WBR, WBR_balance_controller)
attach_module_functions(WBR, WBR_state_estimator)
attach_module_functions(WBR, WBR_velocity_controller)