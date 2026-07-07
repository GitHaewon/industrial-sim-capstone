"""Shared labels and mappings for the factory manager package."""

ITEM_CLASS = {
    'item_a_red_cube': 'A',
    'item_b_green_cylinder': 'B',
    'item_c_blue_hex': 'C',
}

DESTINATION_BY_CLASS = {
    'A': '왼쪽 출하 도크 A',
    'B': '중앙 출하 도크 B',
    'C': '오른쪽 출하 도크 C',
}

FACTORY_LABELS = {
    'IDLE': '공정 대기',
    'CONVEYOR_RUNNING': '컨베이어 운전',
    'ITEM_READY': '제품 감지',
    'PICKING': '흡착 픽업',
    'BOX_READY': '박스 적재 완료',
    'AMR_MOVING': 'Nav2 출하 운송',
    'DELIVERED': '배송 완료',
    'DELIVERY_FAILED': '배송 실패',
    'RESETTING': '다음 생산 준비',
}
