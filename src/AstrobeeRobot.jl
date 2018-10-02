__precompile__()

module AstrobeeRobot

using RigidBodyDynamics
using StaticArrays

export Astrobee,
  packagepath,
  urdfpath

packagepath() = joinpath(@__DIR__, "..", "deps")
urdfpath() = joinpath(packagepath(), "Astrobee", "astrobee.urdf")

mutable struct Astrobee{T}
  mechanism::Mechanism{T}
  basejoint::Joint
  body::RigidBody

  function Astrobee{T}(;floating=true) where T
    mechanism = RigidBodyDynamics.parse_urdf(T,urdfpath())

    #create floating joint
    body = findbody(mechanism, "body") 
    basejoint = joint_to_parent(body,mechanism)
    if floating
     println("instantiated a floating joint")
     floatingjoint = Joint(basejoint.name, frame_before(basejoint), frame_after(basejoint), QuaternionFloating{T}())
     replace_joint!(mechanism, basejoint, floatingjoint)
     basejoint=floatingjoint
   end

  new{T}(mechanism, basejoint, body)
  end
end
Astrobee(::Type{T} = Float64; kwargs...) where {T} = Astrobee{T}(; kwargs...)

function __init__()
  if !isfile(urdfpath())
    error("Could not find $(urdfpath()). Please run `import Pkg; Pkg.build(\"Astrobee\")`.")
  end
end

end # module
