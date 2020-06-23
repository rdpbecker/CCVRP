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

function pathEdges(i,m)
    if i == 1
        return m
    end
    return 1
end

function cut(S,V)
    V_S = setdiff(V,S)
    cutList = []
    for elem in S
        for elem2 in V_S
            push!(cutList,[elem,elem2])
        end
    end
    return cutList
end

function allSubs(inputList,outputList=[],setList=[],index=1)
    if index == size(inputList)[1]+1
        push!(setList,outputList)
        return
    end
    allSubs(inputList,deepcopy(outputList),setList,index+1)
    new = deepcopy(outputList)
    push!(new,inputList[index])
    allSubs(inputList,new,setList,index+1)
end

function solve_tsp(; verbose = true)

    m = 2

    # Define the vertex names
    verts = ["A","B","C","D","E"]

    # Distance between the pairs of vertices
    distance = [1000000 1 1000 1000 1;
        1 1000000 9 9 9;
        1000 9 1000000 1 9;
        1000 9 1 1000000 9;
        1 9 9 9 1000000]

    num_verts = length(verts)

    powset = []
    allSubs(1:num_verts,[],powset,1)
    powset = [powset[i] for i in 2:2^num_verts-1]
    cuts = []
    for elem in powset
        push!(cuts,cut(elem,1:num_verts))
    end

    model = Model(with_optimizer(Gurobi.Optimizer))

    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts], Bin)

    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,1:i-1]) + sum(y[i,i+1:num_verts]) == pathEdges(i,m)
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[1:j-1,j]) + sum(y[j+1:num_verts,j]) == pathEdges(j,m)
    )

    # flow out
    @constraint(model, flowOut[k in 1:num_verts],
        sum(x[1,:,k]) == pathEdges(k,m)
    )

    # flow cons
    @constraint(model, flowCons[i in 1:num_verts, k in 1:num_verts],
        sum(x[i,:,k])-sum(x[:,i,k]) == numOnPath(i,k)
    )

    # Cut constraint
    @constraint(model, cutCons[i in 1:2^num_verts-2],
        sum([y[edge[1],edge[2]]+y[edge[2],edge[1]] for edge in cuts[i]]) >= 2
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
