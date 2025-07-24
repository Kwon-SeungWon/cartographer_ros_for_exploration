#!/usr/bin/env python3

import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random
import sys  # 프로그램 종료를 위한 모듈

# 1️⃣ 웨이포인트 그래프 생성
G = nx.Graph()
waypoints = {
    0: (0, 0), 1: (3, 4), 2: (6, 6), 3: (9, 3), 4: (5, -2), 5: (2, -3)
}
edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0), (1, 4)]

G.add_nodes_from(waypoints.keys())
G.add_edges_from(edges)

# 2️⃣ 노드 위치 저장
pos = waypoints.copy()

# 3️⃣ 로봇 초기 위치 설정 (여기서는 그래프 내부로 가정)
robot_node = "robot"
robot_initial_pos = np.array([3, 2])  # 그래프 내부의 임의 위치

# 4️⃣ 로봇이 그래프 내부인지 확인
def is_point_on_any_edge(point, pos, edges, tolerance=0.1):
    """
    주어진 점이 그래프의 어떤 엣지 위에 있는지 확인.
    """
    for edge in edges:
        a, b = np.array(pos[edge[0]]), np.array(pos[edge[1]])
        proj, t = project_point_on_line(point, a, b)
        dist = np.linalg.norm(proj - point)
        if dist < tolerance:  # 일정 거리 이내라면 엣지 위에 있다고 판단
            return edge, proj, t
    return None, None, None

def project_point_on_line(p, a, b):
    """
    점 p를 선분 (a, b)에 투영하여 선분 위의 최근접 점을 찾음.
    """
    ap = p - a
    ab = b - a
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(t, 0, 1)  # 0 <= t <= 1 범위 내에서 보정
    return a + t * ab, t  # 투영된 점, 보간된 비율 t 반환

def find_closest_edge(pos, point):
    """
    그래프의 모든 엣지를 확인하여, 주어진 점에서 가장 가까운 엣지를 찾음.
    """
    min_dist = float('inf')
    best_edge, projected_pos, best_t = None, None, 0
    for edge in G.edges:
        a, b = np.array(pos[edge[0]]), np.array(pos[edge[1]])
        proj, t = project_point_on_line(point, a, b)
        dist = np.linalg.norm(proj - point)
        if dist < min_dist:
            min_dist, best_edge, projected_pos, best_t = dist, edge, proj, t
    return best_edge, projected_pos, best_t

# 5️⃣ 로봇이 이미 엣지 위에 있는지 검사
current_edge, robot_pos, t = is_point_on_any_edge(robot_initial_pos, pos, G.edges)

# 6️⃣ 그래프 내부에 없다면 가장 가까운 엣지로 보정
if current_edge is None:
    current_edge, robot_pos, t = find_closest_edge(pos, robot_initial_pos)

# 7️⃣ 로봇을 그래프에 추가
pos[robot_node] = robot_pos
G.add_node(robot_node)

# 8️⃣ 그래프 시각화 설정
fig, ax = plt.subplots(figsize=(6, 6))

def update(frame):
    global robot_pos, current_edge, t
    ax.clear()

    start_node, end_node = current_edge
    start_pos, end_pos = np.array(pos[start_node]), np.array(pos[end_node])

    t += 0.05
    if t >= 1.0:
        t = 0.0
        current_node = end_node
        possible_edges = [e for e in G.edges(current_node) if start_node not in e and "robot" not in e]
        if possible_edges:
            next_edge = random.choice(possible_edges)
            current_edge = (current_node, next_edge[1] if next_edge[0] == current_node else next_edge[0])
        start_node, end_node = current_edge
        start_pos, end_pos = np.array(pos[start_node]), np.array(pos[end_node])

    robot_pos = (1 - t) * start_pos + t * end_pos
    pos[robot_node] = robot_pos
    G.remove_edges_from(list(G.edges(robot_node)))
    G.add_edge(robot_node, start_node)
    G.add_edge(robot_node, end_node)

    nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray', ax=ax)
    ax.scatter(*robot_pos, color='red', s=200, label="Robot")
    ax.legend()
    ax.set_title("Robot Starts Inside or Outside Correctly")

ani = animation.FuncAnimation(fig, update, frames=200, interval=100, repeat=True)

# 9️⃣ 창 닫을 때 애니메이션 중지 및 강제 종료
def on_close(event):
    ani.event_source.stop()
    plt.close(fig)
    sys.exit(0)

fig.canvas.mpl_connect("close_event", on_close)

plt.show()