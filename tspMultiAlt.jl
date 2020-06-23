using JuMP, Gurobi, JSON 

graph_num = "1"
num_vehicles = 2
capacity = 3

function convertToArray(arr)
    return [arr[i][j] for i in 1:length(arr), j in 1:length(arr[1])]
end

function numOnPath(i,k)
    if k == 1
        return 0
    end
    if i == k
        return -1
    end
    if i == 1
        return 1
    end
    return 0
end

function pathEdges(i,m)
    if i == 1
        return m
    end
    return 1
end

function solve_tsp(; verbose = true)

    # Load the adjacency matrix
    distance = open(string("Graphs/graph",graph_num,".json")) do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    distance = convertToArray(distance)

    # Create the vertex array
    num_verts = size(distance,1)
    verts = 1:num_verts

    model = Model(with_optimizer(Gurobi.Optimizer))

    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts], Bin)

    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,:]) == pathEdges(i,num_vehicles)
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[:,j]) == pathEdges(j,num_vehicles)
    )

    # flow out
    @constraint(model, flowOut[k in 1:num_verts],
        sum(x[1,:,k]) == pathEdges(k,num_vehicles)
    )

    # flow cons
    @constraint(model, flowCons[i in 1:num_verts, k in 1:num_verts],
        sum(x[i,:,k])-sum(x[:,i,k]) == numOnPath(i,k)
    )

    # couple the ys and xs
    @constraint(model, 
        couple[i in 1:num_verts, j in 1:num_verts, k in 1:num_verts], 
        x[i,j,k]-y[i,j] <= 0
    )

    @objective(model, Min, sum(distance[i,j]*y[i,j] 
        for i in 1:num_verts, j in 1:num_verts)
    )

    JuMP.optimize!(model)

    if verbose
        println("RESULTS:")
        for i in 1:num_verts
            for j in 1:num_verts
                println("Vertex $(i) to vertex $(j) = $(JuMP.value(y[i,j]))")
                for k in 1:num_verts
                    println("On path to $(k): $(JuMP.value(x[i,j,k]))")
                end
                println("")
            end
        end
    end

end

solve_tsp()
