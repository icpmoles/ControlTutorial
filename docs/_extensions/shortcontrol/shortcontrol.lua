function CodeBlock(code)
  if (code.classes:includes("control")) then
   
    targetType = pandoc.utils.stringify(code.attributes["target"])
    local codePayload = pandoc.utils.stringify(code.text)
    local IfrrameUrl = "https://pages.icpmol.es/ControlChallenges/index.html"
    local url = IfrrameUrl .. "?target=" .. targetType .. '&code=' .. quarto.base64.encode(codePayload)
    local cssStyle = 'width:100%;height:90vh;border-width:3px;border-style:dashed'

    linkString = "Open Simulation"

    if quarto.doc.isFormat('html')  then
      return pandoc.Div({
        pandoc.RawBlock('html',  '<iframe src="' .. url .. '" style=' .. cssStyle .. '></iframe>'),
        pandoc.Link(linkString, url)
      },{class = 'jsSimulation'})
    else
      return pandoc.Div({
        code,
        pandoc.Link(linkString, url)
      })
    end
  end
end
