from conveyor_control.random_spawner import generate_layout


def test_layout_is_reproducible_and_contains_each_class():
    first = generate_layout(42)
    second = generate_layout(42)

    assert first == second
    assert {item.item_class for item in first} == {'A', 'B', 'C'}


def test_layout_respects_arrival_order_and_gap_limits():
    layout = generate_layout(17, minimum_gap=0.9, maximum_gap=1.2)
    gaps = [
        layout[index].x - layout[index + 1].x
        for index in range(len(layout) - 1)
    ]

    assert all(0.9 <= gap <= 1.2 for gap in gaps)
    assert all(
        layout[index].x > layout[index + 1].x
        for index in range(len(layout) - 1)
    )


def test_different_seeds_can_change_layout():
    layouts = {
        tuple(item.item_class for item in generate_layout(seed))
        for seed in range(10)
    }

    assert len(layouts) > 1
