if [ -z "$2" ]; then
    echo "Usage: ./run.sh <input file> <output number>"
    exit 1
fi
julia $1.jl > out$1$2
