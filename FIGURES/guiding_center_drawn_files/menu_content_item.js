define(["require","exports","tslib","classnames","modules/clean/react/css","react","spectrum/popover"],(function(e,t,n,a,o,s,c){"use strict";Object.defineProperty(t,"__esModule",{value:!0}),a=n.__importDefault(a),s=n.__importStar(s),t.MenuContentItem=function(e){var n=e.openOption,a=e.description;return s.default.createElement(c.PopoverContentItem,{value:n,className:"app-action-row"},s.default.createElement(t.MenuContentElement,{text:n.text,iconUrl:n.iconUrl,description:a}))},t.MenuContentElementComponent=function(e){var t=e.text,n=e.iconUrl,o=e.description;return s.default.createElement(s.Fragment,null,s.default.createElement("div",{className:"app-action-row-content"},n?s.default.createElement("img",{alt:"",src:n,className:"extensions-popover-icon"}):null,s.default.createElement("span",{className:a.default("app-action-legacy-option-text",{"extensions-no-icon":!n})},t)),o?s.default.createElement("div",{className:"app-action__inline-value-description"},o):null)},t.MenuContentElement=o.requireCssWithComponent(t.MenuContentElementComponent,["/static/css/app_actions/index-vflL7QoxZ.css","/static/css/extensions/index-vflRLfsLb.css"])}));
//# sourceMappingURL=menu_content_item.min.js-vflUOpg95.map