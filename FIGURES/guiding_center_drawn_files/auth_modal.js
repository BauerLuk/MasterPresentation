define(["require","exports","tslib","react","modules/clean/react/modal","modules/clean/react/extensions/auth_body_v2","modules/clean/ux_analytics_modal_tracking"],(function(e,a,t,l,o,n,s){"use strict";Object.defineProperty(a,"__esModule",{value:!0}),l=t.__importDefault(l),a.AuthModal=function(e){return l.default.createElement(o.Modal,{className:"extensions-auth-modal-v2",ariaLabel:"Access Modal",displayCloseButton:!0,clickOutToClose:!1,style:"clean",onDismiss:e.doCancel},l.default.createElement(s.UXAnalyticsModalTracking,{id:"EXTENSIONS_AUTH_MODAL_V2"}),l.default.createElement(n.ExtensionsAuthBodyV2,t.__assign({},e)))},a.showAuthModal=function(e,t,n,s,c,d){o.Modal.showInstance(l.default.createElement(a.AuthModal,{actionId:e,appName:t,iconUrl:n,fileId:s,fileName:c,doAuth:d,doCancel:o.Modal.close}))}}));
//# sourceMappingURL=auth_modal.min.js-vfl6uWs8n.map