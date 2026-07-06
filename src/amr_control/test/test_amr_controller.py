from amr_control.navigation_policy import retry_allowed


def test_retry_can_be_disabled():
    assert not retry_allowed(1, 0)


def test_retry_allowed_up_to_configured_limit():
    assert retry_allowed(1, 2)
    assert retry_allowed(2, 2)


def test_retry_stops_after_configured_limit():
    assert not retry_allowed(3, 2)
