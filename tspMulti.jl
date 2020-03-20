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

function maxDemand(S,demands)
    return maximum([sum([demands[i][j] for j in S]) for i in 1:length(demands)])
end

function minCutVal(S,demands,capacity)
    return 2*ceil(maxDemand(S,demands)/(2*capacity))
end

function solve_tsp(; verbose = true)

    graph_num = "1"
    num_vehicles = 2
    capacity = 2

    # Define the vertex names
    verts = ["A","B","C","D","E"]

    # Distance between the pairs of vertices

    distance = open(string("Graphs/graph",graph_num,".json")) do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    distance = convertToArray(distance)

    demands = open("demands.json") do f
        txt = read(f,String)
        JSON.parse(txt)
    end

    num_verts = length(verts)

    powset = []
    allSubs(2:num_verts,[],powset,1)
    powset = [powset[i] for i in 2:2^(num_verts-1)-1]
    cuts = []
    demandsByCut = []
    for elem in powset
        push!(cuts,cut(elem,1:num_verts))
        push!(demandsByCut,maxDemand(elem,demands))
    end

    model = Model(with_optimizer(Gurobi.Optimizer))

    # Add the y variables
    @variable(model, y[1:num_verts,1:num_verts], Bin)

    # Add the x variables
    @variable(model, x[1:num_verts,1:num_verts,1:num_verts], Bin)

    # in d
    @constraint(model, indCon[i in 1:num_verts],
        sum(y[i,1:i-1]) + sum(y[i,i+1:num_verts]) == pathEdges(i,num_vehicles)
    )

    # out d
    @constraint(model, outdCon[j in 1:num_verts],
        sum(y[1:j-1,j]) + sum(y[j+1:num_verts,j]) == pathEdges(j,num_vehicles)
    )

    # flow out
    @constraint(model, flowOut[k in 1:num_verts],
        sum(x[1,:,k]) == pathEdges(k,num_vehicles)
    )

    # flow cons
    @constraint(model, flowCons[i in 1:num_verts, k in 1:num_verts],
        sum(x[i,:,k])-sum(x[:,i,k]) == numOnPath(i,k)
    )

    # Cut constraint
    @constraint(model, cutCons[i in 1:2^(num_verts-1)-2],
        sum([y[edge[1],edge[2]]+y[edge[2],edge[1]] for edge in cuts[i]]) >= minCutVal(powset[i],demands,capacity)
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
