-- main_enhanced_filter.lua – kompatibel ohne pandoc.List

-- Attribute/Classes sicherstellen (als einfache Tabellen)
local function ensure_attr(el)
  if not el.attr then el.attr = pandoc.Attr() end
  if not el.attr.classes then el.attr.classes = {} end
end

local function has_class(el, cls)
  ensure_attr(el)
  for _, c in ipairs(el.attr.classes) do
    if c == cls then return true end
  end
  return false
end

local function add_class(el, cls)
  ensure_attr(el)
  if not has_class(el, cls) then
    table.insert(el.attr.classes, cls)
  end
end

-- Tabellen: Klassen ergänzen (duplikatfrei)
function Table(tbl)
  add_class(tbl, 'table')
  add_class(tbl, 'table-compact')
  return tbl
end

-- CodeBlock: Mermaid unverändert lassen
function CodeBlock(cb)
  ensure_attr(cb)
  for _, c in ipairs(cb.attr.classes) do
    if string.lower(c) == 'mermaid' then
      return cb
    end
  end
  return cb
end

-- Inline-Math kapseln; DisplayMath unverändert
function Math(el)
  if el.mathtype == 'InlineMath' then
    local html = '<span class="math math-inline">$' .. el.text .. '$</span>'
    return pandoc.RawInline('html', html)
  end
  return el
end

-- Prüft, ob Block ausschließlich EIN DisplayMath enthält
local function sole_display_math_block(blk)
  if blk.t ~= 'Para' and blk.t ~= 'Plain' then return nil end
  local inlines = blk.content
  if #inlines ~= 1 then return nil end
  local x = inlines[1]
  if x.t == 'Math' and x.mathtype == 'DisplayMath' then
    return x.text
  end
  return nil
end

-- Reine DisplayMath-Absätze → HTML-Block
function Blocks(blocks)
  local out = {}
  for _, blk in ipairs(blocks) do
    local dm = sole_display_math_block(blk)
    if dm then
      table.insert(out, pandoc.RawBlock('html', '<div class="math math-display">$$' .. dm .. '$$</div>'))
    else
      table.insert(out, blk)
    end
  end
  return out
end

-- Hauptcontainer nur einmal einfügen
local main_container_start = '<div class="main-container">'
local main_container_end   = '</div>'

function Pandoc(doc)
  local first = doc.blocks[1]
  local last  = doc.blocks[#doc.blocks]
  local has_start = first and first.t == 'RawBlock' and first.format == 'html'
                    and first.text:match('main%-container')
  local has_end   = last and last.t == 'RawBlock' and last.format == 'html'
                    and last.text:match('^%s*</div>%s*$')
  if not has_start then
    table.insert(doc.blocks, 1, pandoc.RawBlock('html', main_container_start))
  end
  if not has_end then
    table.insert(doc.blocks, pandoc.RawBlock('html', main_container_end))
  end
  return doc
end

return {
  Table    = Table,
  CodeBlock= CodeBlock,
  Math     = Math,
  Blocks   = Blocks,
  Pandoc   = Pandoc
}
