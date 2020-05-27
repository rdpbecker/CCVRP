using JuMP, Gurobi, JSON

function convertToArray(arr)
    return [arr[i][j] for i in 1:length(arr[1]), j in 1:length(arr)]
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

function solve_tsp(; verbose = true)

    graph_num = "1"

    # Define the vertex names
    verts = ["A","B","C","D","E"]

    # Distance between the pairs of vertices

    distance = open(string("Graphs/graph",graph_num,".json")) do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    distance = convertToArray(distance)

    num_verts = length(verts)

    model = Model(with_optimizer(Gurobi.Optimizer))

    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts], Bin)

    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,1:num_verts]) == 1
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[1:num_verts,j]) == 1
    )

    # flow out
    @constraint(model, flowOut[k in 1:num_verts],
        sum(x[1,:,k]) == 1
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
