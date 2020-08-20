define(["require","exports","tslib","react","dig-components/menu","modules/clean/react/app_actions/byline","modules/clean/react/css","modules/clean/react/extensions/cloud_docs_compat","modules/clean/react/extensions/common","modules/clean/react/file_viewer/open_button/types","modules/clean/react/sprite","modules/constants/file_viewer"],(function(e,t,n,o,i,l,a,c,r,s,u,p){"use strict";Object.defineProperty(t,"__esModule",{value:!0});var d=(function(e){function t(){return null!==e&&e.apply(this,arguments)||this}return n.__extends(t,e),t.prototype.render=function(){var e=this.props,t=e.unityOptions,n=e.legacyCloudEditorOptions,a=e.cloudEditorAppActions,d=e.bylines,m=e.currentSession,y=n.map((function(e){var t=e.type===s.OpenButtonAction.OPEN_WITH_CLOUD_DOC?"cloud_doc":"wopi";return o.default.createElement(i.Menu.ActionItem,{key:e.text,value:e,withLeftAccessory:e.spriteName?o.default.createElement(u.Sprite,{group:"web",name:e.spriteName,alt:""}):o.default.createElement("img",{alt:"",src:e.iconUrl,width:24})},o.default.createElement(l.ExtensionsBylineTooltip,{bylines:d[t],onDidShow:r.handleShowByline(t,m),key:e.text},o.default.createElement("span",null,e.text)))})),f=t.map((function(e){return o.default.createElement(i.Menu.ActionItem,{key:e.text,value:e,withLeftAccessory:o.default.createElement(u.Sprite,{group:"web",name:e.spriteName||"",alt:""})},o.default.createElement(l.ExtensionsBylineTooltip,{bylines:d.unity,onDidShow:r.handleShowByline("unity",m),key:e.text},o.default.createElement("span",null,e.text)))})),E=!1,_=a.map((function(e){E||(E=c.isWopiAction(e.appAction));var t=e.appAction.handler.editor_name;return o.default.createElement(i.Menu.ActionItem,{key:e.appAction.description,value:e,withLeftAccessory:o.default.createElement("img",{alt:"",src:e.appAction.icon.url,width:24})},o.default.createElement(l.ExtensionsBylineTooltip,{bylines:d[t],onDidShow:r.handleShowByline(t,m),key:e.appAction.description},o.default.createElement("span",null,e.appAction.description)))})),h=f.length>0||y.length>0||_.length>0;return"ON"===p.OPEN_WITH_DROP_DOWN_ORDER_CHANGE&&E?h&&o.default.createElement(i.Menu.Segment,{key:"open-options"},_,f,y):h&&o.default.createElement(i.Menu.Segment,{key:"open-options"},f,y,_)},t})((o=n.__importDefault(o)).default.Component);t.UnityAndCloudEditors=a.requireCssWithComponent(d,["/static/css/app_actions/index-vflL7QoxZ.css"])}));
//# sourceMappingURL=unity_and_cloud_editors.min.js-vflPCEEl4.map