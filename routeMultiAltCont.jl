using JuMP, Gurobi, JSON 

graph_num = "11"
demands_num = "40"
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

function sumNotEq(list1,list2,i)
    sum = 0
    for j in 1:length(list1)
        if j != i
            sum = sum + list1[j]*list2[j]
        end
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

    # Load the demand scenarios
    demands = open(string("Demands/demands",demands_num,".json")) do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    demands = convertToArray(demands)

    # Create the vertex array
    num_verts = size(distance,1)
    verts = 1:num_verts
    num_scens = size(demands,1)

    model = Model(with_optimizer(Gurobi.Optimizer))

    # Add the y variables
    @variable(model, 0 <= y[1:num_verts,1:num_verts] <= 1)

    # Add the x variables
    @variable(model, 0 <= x[1:num_verts,1:num_verts,1:num_verts] <= 1)

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

    ## Scenario variables
    # One if the vertex is served after refill in this scenario, 
    # zero else
    @variable(model, 0 <= w[1:num_scens,1:num_verts] <= 1)

    # Equal to y_ij w_js (1-w_is)
    @variable(model, 0 <= z[1:num_verts,1:num_verts,1:num_scens] <= 1)

    ## Scenario constraints
    # In these two constraints, we determine whether the customer
    # at a given vertex is served before or after the refill
    @constraint(model,
        serve[k in 2:num_verts, s in 1:num_scens],
        sum(sumNotEq(demands[s,2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - w[s,k]*capacity <= capacity-1
    )

    @constraint(model,
        servecomp[k in 2:num_verts, s in 1:num_scens],
        sum(sumNotEq(demands[s,2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - w[s,k]*capacity >= 0 
    )

    @constraint(model,
        wone[s in 1:num_scens], w[s,1] == 0
    )

    @constraint(model,
        binaryMaxOne[i in 1:num_verts, j in 1:num_verts, s in 1:num_scens],
        z[i,j,s] <= y[i,j]
    )

    @constraint(model,
        binaryMaxTwo[i in 1:num_verts, j in 1:num_verts, s in 1:num_scens],
        z[i,j,s] <= w[s,j]
    )

    @constraint(model,
        binaryMaxThree[i in 1:num_verts, j in 1:num_verts, s in 1:num_scens],
        z[i,j,s] <= 1-w[s,i]
    )

    @constraint(model,
        binaryMin[i in 1:num_verts, j in 1:num_verts, s in 1:num_scens],
        z[i,j,s] >= y[i,j] + w[s,j] - w[s,i] - 1
    )

    @objective(model, Min, 
        sum(distance[i,j]*y[i,j] for i in 1:num_verts, j in 1:num_verts) +
        (1/num_scens)*sum(z[i,j,s]*(distance[j,1]+distance[1,j]) for s in 1:num_scens, i in 1:num_verts, j in 1:num_verts)
    )

    JuMP.optimize!(model)

    if termination_status(model) == MOI.INFEASIBLE_OR_UNBOUNDED
        return
    end

    if verbose
        println("RESULTS:")
        println("Path variables:")
        for i in 1:num_verts
            for j in 1:num_verts
                println("Vertex $(i) to vertex $(j) = $(JuMP.value(y[i,j]))")
                for k in 1:num_verts
                    println("On path to $(k): $(JuMP.value(x[i,j,k]))")
                end
                println("")
            end
        end
        println("Scenario variables:")
        for s in 1:num_scens
            println("Scenario $(demands[s,:])")
            for i in 2:num_verts
                println("Vertex $(i)")
                println("Served in second pass: $(JuMP.value(w[s,i]))")
                for j in 2:num_verts
                    println("Goes back to depot: $(JuMP.value(z[i,j,s]))")
                end
            end
            println("")
        end
    end
end

solve_tsp()
