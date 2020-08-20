define(["require","exports","tslib","react","classnames","spectrum/file_icon","spectrum/icon_content","spectrum/popover","modules/clean/em_string","modules/clean/react/breadcrumb/constants","modules/clean/react/breadcrumb/utils","modules/clean/react/css","modules/clean/react/flag","modules/clean/react/sprite","modules/core/i18n"],(function(e,t,r,n,a,o,l,s,i,u,c,m,p,f,d){"use strict";Object.defineProperty(t,"__esModule",{value:!0}),n=r.__importDefault(n),a=r.__importDefault(a);var b=(function(e){function t(){var t=null!==e&&e.apply(this,arguments)||this;return t.onSegmentClicked=function(e,r){t.onCrumbClicked(e,r)},t.onMenuClicked=function(e,r,n){t.onCrumbClicked(e,r)},t}return r.__extends(t,e),Object.defineProperty(t.prototype,"crumbs",{get:function(){return r.__spreadArrays(this.props.overflowCrumbs,this.props.inlineCrumbs)},enumerable:!0,configurable:!0}),Object.defineProperty(t.prototype,"lastCrumb",{get:function(){return this.props.inlineCrumbs[this.props.inlineCrumbs.length-1]},enumerable:!0,configurable:!0}),t.prototype.onCrumbClicked=function(e,t){this.props.onChange&&this.props.onChange(e,t,this.crumbs.indexOf(t))},t.prototype.render=function(){var e=this;return n.default.createElement("span",null,n.default.createElement("h1",{className:"ax-visually-hidden"},this.lastCrumb.label),n.default.createElement("nav",{className:"breadcrumb-trail u-l-fl","aria-label":d.intl.formatMessage({id:"j2wHLa",defaultMessage:"Folder Hierarchy"}),role:"navigation"},this.props.overflowCrumbs.length?n.default.createElement(v,{crumbs:this.props.overflowCrumbs,onChange:this.onMenuClicked,overflowAttachDirection:this.props.overflowAttachDirection,overflowAttachPosition:this.props.overflowAttachPosition}):null,this.props.inlineCrumbs.map((function(t,r){return n.default.createElement(_,{key:r,crumb:t,isLastCrumb:t===e.lastCrumb,onClick:e.onSegmentClicked,maxWidth:c.getMaxInlineTrailWidth(!!e.props.overflowCrumbs.length)})}))))},t.defaultProps={overflowCrumbs:[]},t})(n.default.Component),h=m.requireCssWithComponent(b,["/static/css/breadcrumbs-vflpOfyDp.css"]);t.BreadcrumbTrail=h,t.SmartBreadcrumbTrail=function(e){var t=c.splitCrumbs(e.crumbs,c.getMaxInlineTrailWidth(!0)),r=t.overflowCrumbs,a=t.inlineCrumbs;return n.default.createElement(h,{overflowCrumbs:r,inlineCrumbs:a,onChange:e.onChange,overflowAttachDirection:e.overflowAttachDirection,overflowAttachPosition:e.overflowAttachPosition})};var C=(function(e){function t(){return null!==e&&e.apply(this,arguments)||this}return r.__extends(t,e),t.prototype.render=function(){return n.default.createElement(f.Sprite,{group:"web",name:"chevron",className:"breadcrumb-separator",alt:""})},t})(n.default.Component),v=(function(e){function t(){var t=null!==e&&e.apply(this,arguments)||this;return t.onClick=function(e,r){t.props.onChange&&t.props.onChange(e,r,t.props.crumbs.indexOf(r))},t}return r.__extends(t,e),t.prototype.render=function(){var e=this,t=r.__spreadArrays(this.props.crumbs);return t.reverse(),n.default.createElement("span",{className:"breadcrumb-menu-container"},n.default.createElement(s.Popover,null,n.default.createElement(s.PopoverTrigger,{className:"breadcrumb-trigger","aria-label":d.intl.formatMessage({id:"HcOP1H",defaultMessage:"Breadcrumb target"})},n.default.createElement("span",{className:"u-unbutton breadcrumb-overflow-button u-l-fl","aria-label":d.intl.formatMessage({id:"ku7o1P",defaultMessage:"Breadcrumb overflow"})},n.default.createElement(l.IconContent,{name:"folder_dropdown-small"}))),n.default.createElement(s.PopoverContent,{attachment:this.props.overflowAttachDirection,position:this.props.overflowAttachPosition},t.map((function(t){return n.default.createElement(g,{key:t.label,crumb:t,onClick:e.onClick})})))),n.default.createElement(C,null))},t})(n.default.Component),g=(function(e){function t(){var t=null!==e&&e.apply(this,arguments)||this;return t.onClick=function(e){t.props.onClick&&t.props.onClick(e,t.props.crumb)},t}return r.__extends(t,e),t.prototype.render=function(){var e=this.props.crumb,t=i.Emstring.em_snippet(e.label,u.MAX_MENU_ITEM_WIDTH_IN_EMS);return n.default.createElement(s.PopoverContentItem,{className:"breadcrumb-menu-item",href:e.href,onClick:this.onClick},n.default.createElement(p.Flag,{leftAttachment:e.isFile?n.default.createElement(o.FileIcon,{path:e.label}):n.default.createElement(l.IconContent,{name:"folder-small"})},n.default.createElement("div",{className:"breadcrumb-menu-item__text u-pad-left-xs"},t)))},t})(n.default.Component),_=(function(e){function t(){var t=null!==e&&e.apply(this,arguments)||this;return t.onClick=function(e){t.props.onClick&&t.props.onClick(e,t.props.crumb)},t}return r.__extends(t,e),t.prototype.getLabel=function(){return i.Emstring.em_snippet(this.props.crumb.label,this.props.maxWidth)},t.prototype.render=function(){var e=this.getLabel();return this.props.isLastCrumb?n.default.createElement("span",{className:"breadcrumb-segment"},e):n.default.createElement("span",null,n.default.createElement("a",{className:a.default("breadcrumb-segment","ancestor-breadcrumb-segment"),href:this.props.crumb.href,onClick:this.onClick},e),n.default.createElement(C,null))},t})(n.default.Component)}));
//# sourceMappingURL=trail.min.js-vflJMCkOz.map