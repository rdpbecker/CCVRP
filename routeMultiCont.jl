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

    # Load the demand scenarios
    demands = open(string("Demands/demands",demands_num,".json")) do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    # Create the vertex array
    num_verts = size(distance,1)
    verts = 1:num_verts
    num_scens = length(demands)

    model = JuMP.direct_model(Gurobi.Optimizer())

    # Add the y variables
    @variable(model, 0 <= y[1:num_verts,1:num_verts,1:num_vehicles] <= 1)

    # Add the x variables
    @variable(model, 0 <= x[1:num_verts,1:num_verts,1:num_verts,1:num_vehicles] <= 1)

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

    demands = convertToArray(demands)

    ## Scenario variables
    # Index of the vertex on the TSP route
    @variable(model, o[1:num_verts,1:num_vehicles] >= 0)

    # The number of customers on route v
    @variable(model, n[1:num_vehicles] >= 0)

    # One if the vertex is served after refill in this scenario, 
    # zero else
    @variable(model, 0 <= w[1:num_scens,1:num_verts,1:num_vehicles] <= 1)

    # The number of customers not served in this scenario
    @variable(model, nns[1:num_scens,1:num_vehicles] >= 0)

    # Positive correction for determining if we need to refill at
    # this vertex in this scenario
    @variable(model, pplus[1:num_scens,1:num_verts,1:num_vehicles] >= 0)

    # Negative correction for determining if we need to refill at
    # this vertex in this scenario
    @variable(model, pminus[1:num_scens,1:num_verts,1:num_vehicles] >= 0)

    # One if there is positive correction, zero else
    @variable(model, 0 <= sign[1:num_scens,1:num_verts,1:num_vehicles] <= 1)

    # One if we need to refill here, zero else
    @variable(model, 0 <= pays[1:num_scens,1:num_verts,1:num_vehicles] <= 1)

    # In these two constraints, we determine whether the customer
    # at a given vertex is served before or after the refill
    @constraint(model,
        serve[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        sum(demands[s,j]*x[i,j,k,v] for i in 1:num_verts, j in 2:num_verts) 
        - w[s,k,v]*capacity <= capacity-1
    )

    @constraint(model,
        servecomp[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        sum(demands[s,j]*x[i,j,k,v] for i in 1:num_verts, j in 2:num_verts) 
        - w[s,k,v]*capacity >= 0 
    )

    # Determine the index of the given vertex in the TSP route
    @constraint(model,
        ordrout[k in 2:num_verts, v in 1:num_vehicles],
        sum(x[i,j,k,v] for i in 1:num_verts, j in 1:num_verts) 
        - o[k,v] == 0 
    )

    # Set the number of customers on route v
    @constraint(model, numVehicles[v in 1:num_vehicles],
        sum(x[i,j,1,v] for i in 1:num_verts, j in 1:num_verts) 
        - n[v] == 0
    )

    # Set the number of customers not served
    @constraint(model, 
        notserv[s in 1:num_scens, v in 1:num_vehicles],
        sum(w[s,k,v] for k in 2:num_verts) - nns[s,v] == 0
    )

    # Set the positive and negative correction
    @constraint(model, 
        lastserv[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        o[k,v] + nns[s,v] + pplus[s,k,v] - pminus[s,k,v] == n[v]
    )

    # Set the sign variable for this vertex and scenario
    @constraint(model, 
        possign[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        pplus[s,k,v] - num_verts*sign[s,k,v] <= 0
    )

    @constraint(model, 
        negsign[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        pminus[s,k,v] + num_verts*sign[s,k,v] <= num_verts
    )

    # Determine if we need to refill at this vertex in this
    # scenario
    @constraint(model, 
        setpay[k in 2:num_verts, s in 1:num_scens, v in 1:num_vehicles],
        pays[s,k,v] + pplus[s,k,v] + pminus[s,k,v] >= 1
    )

    @objective(model, Min, 
        sum(distance[i,j]*y[i,j,v] for i in 1:num_verts, j in 1:num_verts, v in 1:num_vehicles) +
        (1/num_scens)*sum(2*pays[s,k,v]*distance[k,1] for s in 1:num_scens, k in 1:num_verts, v in 1:num_vehicles)
    )

    JuMP.optimize!(model)

    if verbose
        println("RESULTS:")
        println("Path variables")
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
        println("Scenario variables:")
        for s in 1:num_scens
            println("Scenario $(demands[s,:])")
            for v in 1:num_vehicles
                println("Not served: $(JuMP.value(nns[s,v]))")
                for i in 2:num_verts
                    println("Vertex $(i)")
                    println("Served in second pass: $(JuMP.value(w[s,i,v]))")
                    println("Goes back to depot: $(JuMP.value(pays[s,i,v]))")
                    println("")
                end
            end
            println("")
        end
    end

end

solve_tsp()
