function isRespondable(handle)
    local respondable=false
    local result,param=simGetObjectInt32Parameter(handle, sim_shapeintparam_respondable)
    if result==1 then respondable=param~=0 end
    return respondable
end

function getPureShapeType(handle)
    local objType=simGetObjectType(handle)
    if objType~=sim_object_shape_type then return nil,nil end
    local result,pureType,dimensions=simGetShapeGeomInfo(handle)
    local compound=simBoolAnd32(result,1)~=0
    local pure=simBoolAnd32(result,2)~=0
    local convex=simBoolAnd32(result,4)~=0
    if not pure then pureType=nil end
    return pureType,dimensions
end

local fileToSave=simFileDialog(sim_filedlg_type_save, 'Select ini file to save', simGetStringParameter(sim_stringparam_scene_path), '', 'INI files', 'ini')
if fileToSave==nil then
    simAddStatusbarMessage('User clicked cancel')
    return
end

local file=io.open(fileToSave, "w")

local objNum=0

local selectedObjects=simGetObjectSelection()
local allObjects=simGetObjectsInTree(sim_handle_scene)
if selectedObjects and #selectedObjects==1 then
        allObjects=simGetObjectsInTree(selectedObjects[1])
end
local allIndividualShapesToRemove={}
local visibleLayers=simGetIntegerParameter(sim_intparam_visible_layers)
for obji=1,#allObjects,1 do
    local objType=simGetObjectType(allObjects[obji])
    local objName=simGetObjectName(allObjects[obji])
    local pos=simGetObjectPosition(allObjects[obji],-1)
    local matr=simGetObjectMatrix(allObjects[obji],-1)
    local quat=simGetObjectMatrix(allObjects[obji],-1)
    local pureType,size=getPureShapeType(allObjects[obji])
    local respondable=isRespondable(allObjects[obji])
    local result,color=simGetShapeColor(allObjects[obji],nil,sim_colorcomponent_ambient_diffuse)
    if respondable and pureType~=nil then
        file:write('[object'..objNum..']\n')
        objNum=objNum+1
        file:write('name='..objName..'\n')
        file:write('position_x='..pos[1]..'\n')
        file:write('position_y='..pos[2]..'\n')
        file:write('position_z='..pos[3]..'\n')
        file:write('orientation_quaternion_x='..quat[1]..'\n')
        file:write('orientation_quaternion_y='..quat[2]..'\n')
        file:write('orientation_quaternion_z='..quat[3]..'\n')
        file:write('orientation_quaternion_w='..quat[4]..'\n')
        local result,pureType,dimensions=simGetShapeGeomInfo(allObjects[obji])
        local compound=simBoolAnd32(result,1)~=0
        local pure=simBoolAnd32(result,2)~=0
        local convex=simBoolAnd32(result,4)~=0
        file:write('type=')
        if pure then
            if pureType==sim_pure_primitive_plane then
                file:write('plane')
            elseif pureType==sim_pure_primitive_disc then
                file:write('disc')
            elseif pureType==sim_pure_primitive_cuboid then
                file:write('cuboid')
            elseif pureType==sim_pure_primitive_spheroid then
                file:write('spheroid')
            elseif pureType==sim_pure_primitive_cylinder then
                file:write('cylinder')
            elseif pureType==sim_pure_primitive_cone then
                file:write('cone')
            elseif pureType==sim_pure_primitive_heightfield then
                file:write('heightfield')
            end
        end
        file:write('\n')
        file:write('size_x='..dimensions[1]..'\n')
        file:write('size_y='..dimensions[2]..'\n')
        file:write('size_z='..dimensions[3]..'\n')
        file:write('color_r='..color[1]..'\n')
        file:write('color_g='..color[2]..'\n')
        file:write('color_b='..color[3]..'\n')
        file:write("\n")
    end
end
file:close()
simAddStatusbarMessage('Saved to '..fileToSave)

