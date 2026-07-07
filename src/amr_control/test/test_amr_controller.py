from amr_control.navigation_policy import retry_allowed
from amr_control.amr_controller import AmrController


def test_retry_can_be_disabled():
    assert not retry_allowed(1, 0)


def test_retry_allowed_up_to_configured_limit():
    assert retry_allowed(1, 2)
    assert retry_allowed(2, 2)


def test_retry_stops_after_configured_limit():
    assert not retry_allowed(3, 2)


def test_green_box_route_leaves_conveyor_inflation_zone_first():
    green = next(
        carrier for carrier in AmrController.CARRIERS
        if carrier[1] == 'B'
    )
    escape_pose = green[6]
    first_nav2_goal = green[5][0]

    assert escape_pose[1] <= -2.2
    assert first_nav2_goal[1] <= -3.0
