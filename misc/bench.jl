using CCS: blockModel
using ControlSystems
using BenchmarkTools


##

cs = blockModel.csys(; g = 0, α = 0, μ = 1, τ = 100//5)



@btime poles(cs)
@btime all(real(poles(cs)).<=0)
@btime all(<=(0),real(poles(cs)))
@btime all(i -> real(i)<=0,poles(cs))


@benchmark poles(cs)
@benchmark all(real(poles(cs)).<=0)
@benchmark all(<=(0),real(poles(cs)))
@benchmark all(i -> real(i)<=0,poles(cs))


v = rand(ComplexF64,600).-2

vbig = [zeros(ComplexF64,1000).-1 ; rand(ComplexF64,100).-1.5];

vbig = [zeros(ComplexF64,1000).-1 ; rand(ComplexF64,100).-0.5];

@benchmark real($vbig)

@benchmark real($vbig).<=0.0
@benchmark real.($vbig).<=0.0

@benchmark all(real($vbig).<=0.0)
@benchmark all(<=(0.0),real($vbig))
@benchmark all(i -> real(i)<=0.0,$vbig)


# from https://github.com/JuliaControl/ControlSystems.jl/blob/9165f9c1ae9b824868b60f459d750b646bcdb07a/lib/ControlSystemsBase/src/types/StateSpace.jl#L233C5-L233C56
@benchmark all(real.($vbig) .<= 0)

@benchmark for i in eachindex($vbig)
                if real($vbig[i])>0.0
                    return false
                end
            end

@benchmark isHurwitz($(Ref(vbig))[])

function isHurwitz(v)
    for i in eachindex(v)
        if real(v[i])>0.0
            return false
        end
    end
    return true
end

@code_llvm all(real(vbig).<=0)
@code_llvm all(<=(0),real(vbig))
@code_llvm all(i -> real(i)<=0.0,vbig)



@benchmark mapreduce(i->real(i)<=0.0, &, $vbig)
mapreduce(i->real(i)<=0.0, &, vbighw)
