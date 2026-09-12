import heapq


def a_star(start, goal, obstacles):
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    cost_so_far = {start: 0}

    while open_list:

        current_priority, current = heapq.heappop(open_list)

        if current == goal:
            break

        neighbors = [
            (current[0] + 1, current[1]),
            (current[0] - 1, current[1]),
            (current[0], current[1] + 1),
            (current[0], current[1] - 1)
        ]

        for neighbor in neighbors:

            if neighbor in obstacles:
                continue

            new_cost = cost_so_far[current] + 1

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:

                cost_so_far[neighbor] = new_cost

                # Manhattan distance heuristic
                heuristic = (
                    abs(goal[0] - neighbor[0]) +
                    abs(goal[1] - neighbor[1])
                )

                priority = new_cost + heuristic

                heapq.heappush(open_list, (priority, neighbor))

                came_from[neighbor] = current

    # Reconstruct path
    path = []

    if goal not in came_from and start != goal:
        return []

    current = goal

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()

    return path