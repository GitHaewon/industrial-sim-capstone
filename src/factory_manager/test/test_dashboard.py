from factory_manager.constants import DESTINATION_BY_CLASS, ITEM_CLASS
from factory_manager.dashboard_page import DASHBOARD_HTML


def test_dashboard_contains_core_metrics():
    for element_id in (
        'factory',
        'count',
        'cycle',
        'confidence',
        'class-a',
        'box-a',
        'dock-a',
        'andon-light',
        'andon-text',
        'events',
        'history',
        'steps',
    ):
        assert f'id="{element_id}"' in DASHBOARD_HTML


def test_dashboard_knows_all_item_classes():
    assert set(ITEM_CLASS.values()) == {'A', 'B', 'C'}


def test_dashboard_maps_boxes_to_three_loading_doors():
    assert DESTINATION_BY_CLASS == {
        'A': '왼쪽 출하 도크 A',
        'B': '중앙 출하 도크 B',
        'C': '오른쪽 출하 도크 C',
    }
