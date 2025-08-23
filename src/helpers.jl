using PlotThemes, Colors, Plots
using PlotThemes: expand_palette

"""
    isHurwitz(poles)

Returns true if all the elements are in the LHP, false otherwise
"""
function isHurwitz(poles::Vector)
mapreduce(i->real(i)<=0.0, &, poles)
end


ccs_palette = [
    RGB(([215, 155,   18] / 255)...), # orange
    RGB(([ 49, 103, 133] / 255)...), # sky blue
    RGB(([  0, 158, 115] / 255)...), # blueish green
    RGB(([240, 228,  66] / 255)...), # yellow
    RGB(([  0, 114, 178] / 255)...), # blue
    RGB(([213,  94,   0] / 255)...), # vermillion
    RGB(([204, 121, 167] / 255)...), # reddish purple
    ]

_ccstheme = PlotTheme(Dict([
    :palette => expand_palette(colorant"white", ccs_palette; lchoices = [57], cchoices = [100]),
    :colorgradient => :magma,
    :grid => true,
    :gridalpha => 0.4,
    :linewidth => 1.4,
    :minorgrid => true,
    :minorticks => 5,
    :legend => (90,:outer),
    :fontfamily => "Helvetica",
    :background => RGB(([245, 246, 247] / 255)...),
    :bginside => RGB(([239, 239, 239] / 255)...),
    :framestyle => :box,
    :markersize => 10,
    :markeralpha => 1,
    :thickness_scaling => 1,
    :ywiden => true,
    :markerstrokewidth => 4,
    :markerstrokealpha => 1,
    :markerstrokecolor => :black,
    :markerstrokestyle => :solid,
    :gridlinewidth => 0.7])
)

function setupEnv()
    add_theme(:ccs, _ccstheme)
    theme(:ccs)
end
