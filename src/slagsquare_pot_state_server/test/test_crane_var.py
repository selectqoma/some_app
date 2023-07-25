import sys
import os
sys.path.append(os.path.dirname(sys.path[0]))
from slagsquare_pot_state_server.crane_var import CraneVar

def test_constructor():
    crane_var = CraneVar()
    assert crane_var is not None,\
        "object should be instantiated"

def test_update():
    crane_var = CraneVar()
    max_len = int(1e4)
    for _ in range(max_len):
        crane_var.update(1.)

    # test that the history is of fixed length
    assert len(crane_var.history) == crane_var.n_hist,\
        "history max length exceeded"

    # test values of inner parameters
    assert crane_var.value == 1.,\
        "latest value should be 1."
    assert crane_var.value_d == 0.,\
        "differentiated value should be 0."

if __name__ == "__main__":
    test_constructor()
    test_update()
    print("Everything passed")
