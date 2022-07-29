
"use strict";

let ListWorlds = require('./ListWorlds.js')
let GetAnnotations = require('./GetAnnotations.js')
let EditAnnotationsData = require('./EditAnnotationsData.js')
let PublishMap = require('./PublishMap.js')
let SetKeyword = require('./SetKeyword.js')
let GetAnnotationsData = require('./GetAnnotationsData.js')
let YAMLExport = require('./YAMLExport.js')
let RenameMap = require('./RenameMap.js')
let SetRelationship = require('./SetRelationship.js')
let SaveMap = require('./SaveMap.js')
let DeleteAnnotations = require('./DeleteAnnotations.js')
let YAMLImport = require('./YAMLImport.js')
let ListMaps = require('./ListMaps.js')
let ResetDatabase = require('./ResetDatabase.js')
let SaveAnnotationsData = require('./SaveAnnotationsData.js')
let PubAnnotationsData = require('./PubAnnotationsData.js')
let DeleteMap = require('./DeleteMap.js')

module.exports = {
  ListWorlds: ListWorlds,
  GetAnnotations: GetAnnotations,
  EditAnnotationsData: EditAnnotationsData,
  PublishMap: PublishMap,
  SetKeyword: SetKeyword,
  GetAnnotationsData: GetAnnotationsData,
  YAMLExport: YAMLExport,
  RenameMap: RenameMap,
  SetRelationship: SetRelationship,
  SaveMap: SaveMap,
  DeleteAnnotations: DeleteAnnotations,
  YAMLImport: YAMLImport,
  ListMaps: ListMaps,
  ResetDatabase: ResetDatabase,
  SaveAnnotationsData: SaveAnnotationsData,
  PubAnnotationsData: PubAnnotationsData,
  DeleteMap: DeleteMap,
};
