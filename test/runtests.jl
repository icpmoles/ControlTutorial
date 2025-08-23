using CCS: blockModel
using CCS
using ControlSystems
using Test

cs = blockModel.csys(; g = 0, α = 0, μ = 1, τ = 100//5)

@testset "Nominal Behaviour" begin
    @test controllability(cs).iscontrollable == true
    @test observability(cs).isobservable == true
    @test CCS.isHurwitz(poles(cs))
end


hwpoles = [zeros(ComplexF64,1000).-1 ; rand(ComplexF64,100).-1.5];
nothwpoles = [zeros(ComplexF64,1000).-1 ; rand(ComplexF64,100).-0.5];

@testset "Hurwitz Check" begin
    @test CCS.isHurwitz(nothwpoles) == false
    @test CCS.isHurwitz(-nothwpoles) == false
    @test CCS.isHurwitz(-hwpoles) == false
    @test CCS.isHurwitz(hwpoles) == true
    @test CCS.isHurwitz([ 0.0 ; 0 ; 0.0+0im ; -0-0.0im]) == true
end
