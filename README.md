# Heuristic-Function
function heuristic = euclidean_heuristic(coord_a, coord_b)
    heuristic = sqrt((coord_a(1) - coord_b(1)) ^ 2 + (coord_a(2) - coord_b(2)) ^ 2);
end
 
% A* search algorithm
function path = a_star_search(graph, coords, start, goal)
    open_set = containers.Map('KeyType', 'double', 'ValueType', 'char');
    open_set(start) = start;
 
    came_from = containers.Map('KeyType', 'double', 'ValueType', 'double');
    g_score = containers.Map('KeyType', 'double', 'ValueType', 'double');
    f_score = containers.Map('KeyType', 'double', 'ValueType', 'double');
 
    g_score(start) = 0;
    f_score(start) = euclidean_heuristic(coords(start, :), coords(goal, :));
 
    while ~isempty(open_set)
        [~, current] = min(cell2mat(f_score.values));
 
        if current == goal
            path = [];
            while isKey(came_from, current)
                path = [path, current];
                current = came_from(current);
            end
            path = [path, start];
            path = fliplr(path);
            return;
        end
 
        open_set.remove(current);
 
        neighbors = graph(current);
        for i = 1:length(neighbors)
            neighbor = neighbors{i}(1);
            cost = neighbors{i}(2);
 
            tentative_g_score = g_score(current) + cost;
 
            if ~isKey(g_score, neighbor) || tentative_g_score < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g_score;
                f_score(neighbor) = g_score(neighbor) + euclidean_heuristic(coords(neighbor, :), coords(goal, :));
 
                if ~isKey(open_set, neighbor)
                    open_set(neighbor) = neighbor;
                end
            end
        end
    end
    error("No path found");
end
