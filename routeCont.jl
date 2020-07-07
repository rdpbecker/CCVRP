using JuMP, Gurobi, JSON

graph_num = "11"
demands_num = "40"
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

    # Make the array of vertices
    num_verts = size(distance,1)
    verts = 1:num_verts
    num_scens = size(demands,1)

    model = Model(with_optimizer(Gurobi.Optimizer))

    ## TSP variables
    # Add the y variables
    @variable(model, 0 <= y[1:num_verts,1:num_verts] <= 1)

    # Add the x variables
    @variable(model, 0 <= x[1:num_verts,1:num_verts,1:num_verts] <= 1)

    ## TSP constraints
    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,1:i-1]) + sum(y[i,i+1:num_verts]) == 1
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[1:j-1,j]) + sum(y[j+1:num_verts,j]) == 1
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

    ## Scenario variables
    # Index of the vertex on the TSP route
    @variable(model, o[1:num_verts] >= 0)

    # One if the vertex is served after refill in this scenario, 
    # zero else
    @variable(model, 0 <= w[1:num_scens,1:num_verts] <= 1)

    # The number of customers not served in this scenario
    @variable(model, nns[1:num_scens] >= 0)

    # One if we need to refill here, zero else
    @variable(model, 0 <= pays[1:num_scens,1:num_verts] <= 1)

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

    # Determine the index of the given vertex in the TSP route
    @constraint(model,
        ordrout[k in 2:num_verts],
        sum(sumNotEq([1 for j in 2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - o[k] == 0 
    )

    # Set the number of customers not served
    @constraint(model, notserv[s in 1:num_scens],
        sum(w[s,k] for k in 2:num_verts) - nns[s] == 0
    )

    # Set the positive and negative correction
    @constraint(model, lastserv[k in 2:num_verts, s in 1:num_scens],
        o[k] + nns[s] + pays[s,k] >= num_verts*w[s,k]+1
    )

    @objective(model, Min, 
        sum(distance[i,j]*y[i,j] for i in 1:num_verts, j in 1:num_verts) +
        (1/num_scens)*sum(2*pays[s,k]*distance[k,1] for s in 1:num_scens, k in 1:num_verts)
    )

    JuMP.optimize!(model)

    if termination_status(model) == MOI.INFEASIBLE_OR_UNBOUNDED
        return
    end

    if verbose
        println("RESULTS:")
        println("Path variables:")
        for i in 1:num_verts
            println("Visited number $(JuMP.value(o[i]))")
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
            println("Not served: $(JuMP.value(nns[s]))")
            for i in 2:num_verts
                println("Vertex $(i)")
                println("Served in second pass: $(JuMP.value(w[s,i]))")
                println("Goes back to depot: $(JuMP.value(pays[s,i]))")
                println("")
            end
            println("")
        end
    end

end

solve_tsp()
