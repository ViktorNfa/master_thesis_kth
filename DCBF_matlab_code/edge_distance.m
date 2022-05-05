% Function to get the distances of each edge
function distance = edge_distance(x_vec)
    distance = zeros(length(x_vec)/2, 1);
    for i=1:length(x_vec)/2
        distance(i) = sqrt((x_vec(3) - x_vec(2*i-1))^2+(x_vec(4) - x_vec(2*i))^2);
    end
end