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

function sumSingle(x,v,n)
    sum = 0
    for i in 1:n
        sum = sum + x[1,i,i,v]
    end
    return sum
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

    model = JuMP.direct_model(Gurobi.Optimizer())

    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts,1:num_vehicles], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts,1:num_vehicles], Bin)

    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,1:num_verts,:]) == pathEdges(i,num_vehicles)
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[1:num_verts,j,:]) == pathEdges(j,num_vehicles)
    )

    # continuity
    @constraint(model, cont[i in 1:num_verts,v in 1:num_vehicles],
        sum(y[i,:,v])-sum(y[:,i,v]) == 0
    )

    # flow out total
    @constraint(model, flowOutTotal[k in 1:num_verts],
        sum(x[1,:,k,:]) == pathEdges(k,num_vehicles)
    )

    # flow cons
    @constraint(model, flowCons[i in 1:num_verts, k in 1:num_verts, v in 1:num_vehicles],
        sum(x[i,:,k,v])-sum(x[:,i,k,v]) == numOnPath(i,k)*sum(x[1,:,k,v])
    )

    # depot out
    @constraint(model, depotOut[v in 1:num_vehicles],
        sumSingle(x,v,num_verts) == 1
    )

    # couple the ys and xs
    @constraint(model, 
        couple[i in 1:num_verts, j in 1:num_verts, k in 1:num_verts, v in 1:num_vehicles], 
        x[i,j,k,v]-y[i,j,v] <= 0
    )

    @objective(model, Min, sum(distance[i,j]*y[i,j,v] 
        for i in 1:num_verts, j in 1:num_verts, v in 1:num_vehicles)
    )

    JuMP.optimize!(model)

    if verbose
        println("RESULTS:")
        for i in 1:num_verts
            for j in 1:num_verts
                for v in 1:num_vehicles
                    println("Vertex $(i) to vertex $(j), vehicle $(v): $(JuMP.value(y[i,j,v]))")
                    for k in 1:num_verts
                            println("On path to $(k), vehicle $(v): $(JuMP.value(x[i,j,k,v]))")
                    end
                    println("")
                end
            end
        end
    end

end

solve_tsp()
