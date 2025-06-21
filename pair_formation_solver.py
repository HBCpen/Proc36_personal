import collections

class PairFormationSolver:
    def __init__(self, board):
        self.board = [row[:] for row in board] # Create a copy to avoid modifying the original board outside the class
        self.protected_coords = set()
        self.rows = len(self.board)
        self.cols = len(self.board[0]) if self.rows > 0 else 0
        # self.history = [] # To record board states for debugging/testing - commented out for final version

    # def _record_state(self, operation_name):
    #     # Record current board state and protected coordinates
    #     current_board_copy = [row[:] for row in self.board]
    #     current_protected_copy = set(self.protected_coords)
    #     self.history.append({
    #         "operation": operation_name,
    #         "board": current_board_copy,
    #         "protected_coords": current_protected_copy,
    #         "rows": self.rows,
    #         "cols": self.cols
    #     })

    def solve(self):
        """
        Solves the board layer by layer from outside to inside.
        """
        # self._record_state("Initial") # Commented out
        num_layers = (min(self.rows, self.cols) + 1) // 2
        for layer_offset in range(num_layers):
            if self.rows - 2 * layer_offset < 2 or self.cols - 2 * layer_offset < 2:
                # print(f"Layer {layer_offset} is too small, stopping.") # Commented out
                break
            self._solve_layer(layer_offset)
        # self._record_state("Final") # Commented out
        # print("Final protected_coords:", sorted(list(self.protected_coords))) # Commented out

    def _solve_layer(self, layer_offset):
        """
        Solves a single layer of the board.
        """
        # print(f"\n--- Solving layer: {layer_offset} ---") # Commented out
        # self._print_board(f"Start of layer {layer_offset}") # Commented out

        # 1. Align top row pairs
        # print(f"Step 1 (Layer {layer_offset}): Aligning top row") # Commented out
        self._align_row_pairs_sequentially(0, 0, 1, layer_offset)
        # self._record_state(f"After Align Top Row (L{layer_offset})") # Commented out

        # 2. Rotate 180 degrees
        # print(f"Step 2 (Layer {layer_offset}): Rotating 180 degrees") # Commented out
        self._rotate_entire_board_and_protected_coords(180)
        # self._record_state(f"After Rotate 180 (L{layer_offset})") # Commented out

        # 3. Align (new) top row pairs (original bottom row)
        # print(f"Step 3 (Layer {layer_offset}): Aligning (new) top row (original bottom)") # Commented out
        self._align_row_pairs_sequentially(0, 0, 1, layer_offset)
        # self._record_state(f"After Align Original Bottom Row (L{layer_offset})") # Commented out

        # 4. Rotate 90 degrees (total 180 + 90 = 270 from original layer start)
        # print(f"Step 4 (Layer {layer_offset}): Rotating 90 degrees") # Commented out
        self._rotate_entire_board_and_protected_coords(90)
        # self._record_state(f"After Rotate 90 (L{layer_offset})") # Commented out

        # 5. Align (new) top row, starting from the second cell (original right col)
        # print(f"Step 5 (Layer {layer_offset}): Aligning (new) top row, offset 1 (original right)") # Commented out
        self._align_row_pairs_sequentially(0, 1, 1, layer_offset)
        # self._record_state(f"After Align Original Right Col (L{layer_offset})") # Commented out

        # 6. Rotate 180 degrees (total 270 + 180 = 450 -> 90 from original layer start)
        # print(f"Step 6 (Layer {layer_offset}): Rotating 180 degrees") # Commented out
        self._rotate_entire_board_and_protected_coords(180)
        # self._record_state(f"After Rotate 180 (L{layer_offset}, Total 90 from layer start)") # Commented out

        # 7. Align (new) top row, starting from the second cell (original left col)
        # print(f"Step 7 (Layer {layer_offset}): Aligning (new) top row, offset 1 (original left)") # Commented out
        self._align_row_pairs_sequentially(0, 1, 1, layer_offset)
        # self._record_state(f"After Align Original Left Col (L{layer_offset})") # Commented out

        # Rotate back to the orientation at the START of this layer's processing
        # print(f"Step 8 (Layer {layer_offset}): Rotating 270 degrees to restore layer start orientation") # Commented out
        self._rotate_entire_board_and_protected_coords(270)
        # self._record_state(f"After Restoring Orientation (L{layer_offset})") # Commented out
        # self._print_board(f"End of layer {layer_offset}, orientation restored") # Commented out


    def _align_row_pairs_sequentially(self, row_idx_sub, col_start_idx_sub, col_step, layer_offset):
        """
        Aligns pairs in a specified row (or column after rotation) of the subgrid.
        This function would call a flood fill algorithm.
        For now, it simulates adding pairs to protected_coords.
        """
        actual_row = layer_offset + row_idx_sub
        subgrid_col_start = layer_offset + col_start_idx_sub
        subgrid_width = (self.cols - 2 * layer_offset)
        subgrid_col_end = layer_offset + subgrid_width

        # print(f"  Aligning pairs in current board row {actual_row} (layer {layer_offset}), from col {subgrid_col_start} to {subgrid_col_end -1}") # Commented out

        if not (0 <= actual_row < self.rows):
            # print(f"  Actual row {actual_row} is out of bounds {self.rows}. Skipping alignment.") # Commented out
            return

        current_col = subgrid_col_start
        while current_col + 1 < subgrid_col_end:
            coord1 = (actual_row, current_col)
            coord2 = (actual_row, current_col + 1)

            if not (0 <= current_col < self.cols and 0 <= current_col + 1 < self.cols):
                # print(f"  Column indices {current_col}, {current_col+1} out of bounds {self.cols}. Stopping alignment for this row.") # Commented out
                break

            # print(f"    Protecting pair: ({self.board[actual_row][current_col]} at {coord1}), ({self.board[actual_row][current_col+1]} at {coord2})") # Commented out
            self.protected_coords.add(coord1)
            self.protected_coords.add(coord2)
            current_col += 2
        # print(f"  Protected_coords after _align_row_pairs_sequentially: {sorted(list(self.protected_coords))}") # Commented out


    def _rotate_entire_board_and_protected_coords(self, angle_degrees):
        if angle_degrees == 0:
            return

        old_rows, old_cols = self.rows, self.cols
        new_board_list = []

        if angle_degrees == 90 or angle_degrees == 270:
            self.rows, self.cols = old_cols, old_rows
            new_board_list = [[0] * self.cols for _ in range(self.rows)]
        elif angle_degrees == 180:
            new_board_list = [[0] * self.cols for _ in range(self.rows)]
        else:
            # print(f"Error: Invalid rotation angle {angle_degrees}") # Commented out
            return

        new_protected_coords = set()

        for r_old in range(old_rows):
            for c_old in range(old_cols):
                val = self.board[r_old][c_old]
                r_new, c_new = -1, -1

                if angle_degrees == 90:
                    r_new, c_new = c_old, old_rows - 1 - r_old
                elif angle_degrees == 180:
                    r_new, c_new = old_rows - 1 - r_old, old_cols - 1 - c_old
                elif angle_degrees == 270:
                    r_new, c_new = old_cols - 1 - c_old, r_old

                if 0 <= r_new < self.rows and 0 <= c_new < self.cols:
                    new_board_list[r_new][c_new] = val
                    if (r_old, c_old) in self.protected_coords:
                        new_protected_coords.add((r_new, c_new))
                # else:
                    # print(f"Warning: Rotated coord ({r_new},{c_new}) for ({r_old},{c_old}) is out of new bounds ({self.rows},{self.cols})") # Commented out

        self.board = new_board_list
        self.protected_coords = new_protected_coords
        # self._print_board(f"After rotation {angle_degrees} deg") # Commented out


    def _print_board(self, title="Board State"): # Kept for potential debugging, but not used by tests
        print(f"\n{title} (Rows: {self.rows}, Cols: {self.cols})")
        if not self.board:
            print("  Board is empty.")
            return
        for r_idx, row_val in enumerate(self.board):
            row_str = []
            for c_idx, cell in enumerate(row_val):
                is_protected = '*' if (r_idx, c_idx) in self.protected_coords else ' '
                try:
                    val_str = str(cell)
                except:
                    val_str = '?'
                row_str.append(f"{val_str}{is_protected}")
            print(f"  {' '.join(row_str)}")
        print(f"  Protected: {sorted(list(self.protected_coords))}")


# --- Test/Usage Example ---
def run_solver_test():
    print("--- Running Solver Test ---")
    # Test case 1: 4x4 board
    board1 = [
        [1, 2, 3, 4],
        [5, 6, 7, 8],
        [9,10,11,12],
        [13,14,15,16]
    ]
    print("\nTest Case 1: 4x4 Board")
    solver1 = PairFormationSolver(board1)
    solver1.solve()

    expected_protected_l0 = {
        (0,0), (0,1), (0,2), (0,3),
        (3,0), (3,1), (3,2), (3,3),
        (1,3), (2,3),
        (1,0), (2,0)
    }

    assert solver1.protected_coords == expected_protected_l0, \
        f"Test Case 1 Failed: Expected {expected_protected_l0}, Got {solver1.protected_coords}"
    print("Test Case 1 (4x4 Board) Passed!")

    # Test case 2: 6x6 board
    board2 = [
        [ 1, 2, 3, 4, 5, 6],
        [ 7, 8, 9,10,11,12],
        [13,14,15,16,17,18],
        [19,20,21,22,23,24],
        [25,26,27,28,29,30],
        [31,32,33,34,35,36]
    ]
    print("\nTest Case 2: 6x6 Board")
    solver2 = PairFormationSolver(board2)
    solver2.solve()

    expected_protected_l0_6x6 = {
        (0,0),(0,1), (0,2),(0,3), (0,4),(0,5),
        (5,0),(5,1), (5,2),(5,3), (5,4),(5,5),
        (1,5),(2,5), (3,5),(4,5),
        (1,0),(2,0), (3,0),(4,0)
    }
    expected_protected_l1_6x6 = {
        (1,1),(1,2), (1,3),(1,4),
        (4,1),(4,2), (4,3),(4,4),
        (2,4),(3,4),
        (2,1),(3,1)
    }
    expected_total_protected_6x6 = expected_protected_l0_6x6.union(expected_protected_l1_6x6)

    assert solver2.protected_coords == expected_total_protected_6x6, \
        f"Test Case 2 Failed: Expected {expected_total_protected_6x6}, Got {solver2.protected_coords}"
    print("Test Case 2 (6x6 Board) Passed!")

    # Test case 3: Non-square board 4x6
    board3 = [
        [1,2,3,4,5,6],
        [7,8,9,10,11,12],
        [13,14,15,16,17,18],
        [19,20,21,22,23,24]
    ]
    print("\nTest Case 3: 4x6 Board")
    solver3 = PairFormationSolver(board3)
    solver3.solve()
    expected_protected_l0_4x6 = {
        (0,0),(0,1), (0,2),(0,3), (0,4),(0,5),
        (3,0),(3,1), (3,2),(3,3), (3,4),(3,5),
        (1,5),(2,5),
        (1,0),(2,0)
    }
    expected_protected_l1_4x6 = {
        (1,1),(1,2), (1,3),(1,4),
        (2,1),(2,2), (2,3),(2,4)
    }
    expected_total_protected_4x6 = expected_protected_l0_4x6.union(expected_protected_l1_4x6)

    assert solver3.protected_coords == expected_total_protected_4x6, \
        f"Test Case 3 Failed: Expected {expected_total_protected_4x6}, Got {solver3.protected_coords}"
    print("Test Case 3 (4x6 Board) Passed!")

    print("\nAll tests passed!")

if __name__ == "__main__":
    run_solver_test()
