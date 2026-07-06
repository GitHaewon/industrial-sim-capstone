"""Pure failure-policy helpers for box navigation."""


def retry_allowed(failed_attempts, max_retries):
    """Return whether another attempt is allowed after a failure."""
    return failed_attempts <= max_retries
