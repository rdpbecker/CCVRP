using JuMP, Gurobi

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

    # Define the vertex names
    verts = ["A","B","C","D","E"]

    # Distance between the pairs of vertices
    distance = [1000000 3 4 5 1;
        3 1000000 5 1 2;
        4 5 1000000 2 3;
        5 1 2 1000000 4;
        1 2 3 4 1000000]

    # Capacity
    cap = 2

    # All possible demand scenarios
    demands = [0 0 0 2 2;
    0 0 1 1 2;
    0 0 1 2 1;
    0 1 0 1 2;
    0 1 0 2 1;
    0 1 1 0 2;
    0 1 1 1 1;
    0 1 1 2 0;
    0 1 2 0 1;
    0 1 2 1 0;
    0 2 0 0 2;
    0 2 0 1 1;
    0 2 0 2 0;
    0 2 1 0 1;
    0 2 1 1 0;
    0 2 2 0 0]
    
    num_verts = length(verts)
    num_scens = size(demands,1)

    model = Model(with_optimizer(Gurobi.Optimizer))

    ## TSP variables
    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts], Bin)

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
    @variable(model, w[1:num_scens,1:num_verts], Bin)

    # The number of customers not served in this scenario
    @variable(model, nns[1:num_scens] >= 0)

    # Positive correction for determining if we need to refill at
    # this vertex in this scenario
    @variable(model, pplus[1:num_scens,1:num_verts] >= 0)

    # Negative correction for determining if we need to refill at
    # this vertex in this scenario
    @variable(model, pminus[1:num_scens,1:num_verts] >= 0)

    # One if there is positive correction, zero else
    @variable(model, sign[1:num_scens,1:num_verts], Bin)

    # One if we need to refill here, zero else
    @variable(model, pay[1:num_scens,1:num_verts], Bin)

    ## Scenario constraints
    # In these two constraints, we determine whether the customer
    # at a given vertex is served before or after the refill
    @constraint(model,
        serve[k in 2:num_verts, s in 1:num_scens],
        sum(sumNotEq(demands[s,2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - w[s,k]*cap <= cap
    )

    @constraint(model,
        servecomp[k in 2:num_verts, s in 1:num_scens],
        sum(sumNotEq(demands[s,2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - w[s,k]*cap >= 0 
    )

    # Determine the index of the given vertex in the TSP route
    @constraint(model,
        ordrout[k in 2:num_verts],
        sum(sumNotEq([1 for j in 2:num_verts],x[i,2:num_verts,k],i-1) for i
        in 1:num_verts) - o[k] == 0 
    )

    # Make sure that the index reached first is served first (next
    # four)
    @constraint(model, 
        precik[i in 2:num_verts, k in 2:num_verts, s in 1:num_scens],
        w[s,i] - w[s,k] <= 1-sum(x[i,2:num_verts,k])
    )

    @constraint(model, 
        precjk[j in 2:num_verts, k in 2:num_verts, s in 1:num_scens],
        w[s,j] - w[s,k] <= 1-sum(x[2:num_verts,j,k])
    )

    @constraint(model, 
        precki[i in 2:num_verts, k in 2:num_verts, s in 1:num_scens],
        w[s,k] - w[s,i] <= sum(x[i,2:num_verts,k])
    )

#    @constraint(model, 
#        preckj[j in 2:num_verts, k in 2:num_verts, s in 1:num_scens],
#        w[s,k] - w[s,j] <= sum(x[2:num_verts,j,k])
#    )

    # Make sure that the total demand of customers served in the
    # first pass is less than the capacity
    @constraint(model, twostage[s in 1:num_scens],
        sum(demands[s,k]*w[s,k] for k in 2:num_verts) <= cap
    )

    # Set the number of customers not served
    @constraint(model, notserv[s in 1:num_scens],
        sum(w[s,k] for k in 2:num_verts) - nns[s] == 0
    )

    # Set the positive and negative correction
    @constraint(model, lastserv[k in 2:num_verts, s in 1:num_scens],
        o[k] + nns[s] + pplus[s,k] - pminus[s,k] == num_verts
    )

    # Set the sign variable for this vertex and scenario
    @constraint(model, possign[k in 2:num_verts, s in 1:num_scens],
        pplus[s,k] - num_verts*sign[s,k] <= 0
    )

    @constraint(model, negsign[k in 2:num_verts, s in 1:num_scens],
        pminus[s,k] + num_verts*sign[s,k] <= num_verts
    )

    # Determine if we need to refill at this vertex in this
    # scenario
    @constraint(model, setpay[k in 2:num_verts, s in 1:num_scens],
        pay[s,k] + pplus[s,k] + pminus[s,k] >= 1
    )

    @objective(model, Min, 
        sum(distance[i,j]*y[i,j] for i in 1:num_verts, j in 1:num_verts) +
        (1/num_scens)*sum(2*pay[s,k]*distance[k,1] for s in 1:num_scens, k in 1:num_verts)
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
                println("Adjustors (+/-): $(JuMP.value(pplus[s,i])), $(JuMP.value(pminus[s,i]))")
                println("Sign: $(JuMP.value(sign[s,i]))")
                println("Goes back to depot: $(JuMP.value(pay[s,i]))")
                println("")
            end
            println("")
        end
    end

end

solve_tsp()
