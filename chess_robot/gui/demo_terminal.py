"""Minimal terminal UI helpers for constrained demo-game runs."""

from __future__ import absolute_import


def _fmt_list(values):
    if not values:
        return "-"
    return ", ".join([str(value) for value in values])


class DemoTerminal(object):
    """Simple printer that keeps demo state visible in the terminal."""

    def render_board_state(self, board_state, robot_label):
        print("=" * 72)
        print("Robot perspective: {}".format(robot_label))
        print("FEN: {}".format(board_state.fen()))
        print("ASCII board:")
        print(board_state.ascii())
        print("=" * 72)

    def render_turn_status(
        self,
        changed_squares,
        candidate_move,
        confirmed_human_move,
        movebook_reply,
        primitive_plan,
        execution_status,
        verification_status,
    ):
        print("Changed squares: {}".format(_fmt_list(changed_squares)))
        print("Candidate human move: {}".format(candidate_move or "-"))
        print("Confirmed human move: {}".format(confirmed_human_move or "-"))
        print("Movebook robot reply: {}".format(movebook_reply or "-"))
        print("Physical action plan: {}".format(_fmt_list(primitive_plan)))
        print("Execution status: {}".format(execution_status or "-"))
        print("Verification status: {}".format(verification_status or "-"))
        print("-" * 72)

    def info(self, message):
        print("[demo] {}".format(message))

    def warn(self, message):
        print("[demo][warning] {}".format(message))

    def error(self, message):
        print("[demo][error] {}".format(message))
