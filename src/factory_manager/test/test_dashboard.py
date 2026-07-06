from factory_manager.dashboard_node import ITEM_CLASS
from factory_manager.dashboard_page import DASHBOARD_HTML


def test_dashboard_contains_core_metrics():
    for element_id in (
        'factory',
        'count',
        'cycle',
        'confidence',
        'class-a',
        'box-a',
        'steps',
    ):
        assert f'id="{element_id}"' in DASHBOARD_HTML


def test_dashboard_knows_all_item_classes():
    assert set(ITEM_CLASS.values()) == {'A', 'B', 'C'}
