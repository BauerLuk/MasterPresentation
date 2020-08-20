define(["require","exports","tslib","react","react-intl","spectrum/modal","modules/core/browser_detection","modules/core/i18n","modules/clean/ux_analytics_modal_tracking","modules/constants/page_load","modules/clean/react/css","modules/clean/react/growth/first_task/first_task","modules/clean/react/admin/teams/onboarding/web/constants","modules/clean/growth/smb_funnel/smb_funnel_logger","modules/clean/teams/admin/widgets/invite_modal/invite_modal_first_task/invite_modal_invite_content","modules/clean/teams/admin/widgets/invite_modal/invite_modal_first_task/invite_modal_member_list","modules/clean/teams/admin/widgets/invite_modal/invite_modal_first_task/invite_modal_sidebar","modules/clean/viewer"],(function(e,t,a,n,s,i,l,o,r,m,d,c,u,_,f,v,g,b){"use strict";Object.defineProperty(t,"__esModule",{value:!0}),n=a.__importDefault(n),l=a.__importStar(l);var p=(function(e){function t(){var t=null!==e&&e.apply(this,arguments)||this;return t.focusRef=n.default.createRef(),t}return a.__extends(t,e),t.prototype.componentDidMount=function(){var e={platform:this.props.loggingExtras.is_client?_.Platform.DESKTOP_SSCV:l.is_supported_mobile_browser()?_.Platform.MOBILE_WEB:_.Platform.WEB};_.SMBFunnelLogger.setCommonTags(e),_.SMBFunnelLogger.log(u.AMPMetrics.ACTIVATION_IMM_VIEW)},t.prototype.render=function(){var e=this.props,t=e.open,a=e.tieredAdmin,l=e.memberList,d=e.memberActionsData,u=e.showSidebar,_=e.emails,p=e.sentEmails,I=e.handleOnClose,M=e.onCSVImport,h=e.handleOnSubmit,C=e.handleCSVImport,w=e.renderInputForm,E=e.renderSubmitBtn,S=e.loggingExtras,k=e.showImportCSV,A=e.isInviteLinkEnabled,L=e.growthActSmbMWInvitesActive,T=L?"invite-modal-first-task-mobile":"invite-modal-first-task",y=b.Viewer.get_viewer(),O=y.work_user,V=y.team_id,B=y.team_name;if(!O||!V||!B)return null;var W=O.id,N=1===l.length||L?o.intl.formatMessage({id:"5dI3gl",defaultMessage:"Invite team members"}):o.intl.formatMessage({id:"WzCl9b",defaultMessage:"Manage team members"});return n.default.createElement(s.IntlProvider,{defaultLocale:"en-US",locale:o.localeToBcp47LangTag(m.USER_LOCALE)},n.default.createElement(i.Modal,{className:L?"mw-invites-modal-sheet":"",open:t,ariaLabel:o.intl.formatMessage({id:"Ntdwnu",defaultMessage:"Invite Members"}),displayCloseButton:!0,onRequestClose:function(){p.length&&c.showClinkTooltip(c.TooltipType.Members),I()},overlayClickClose:!0,width:u?960:699,overflowY:!0,focusedElementOnOpen:this.focusRef,appElement:document.body,bodyClassName:"invite-modal "+T+" "+(u?T+"--sidebar-visible":"")},n.default.createElement(r.UXAnalyticsModalTracking,{id:"invite-modal"}),n.default.createElement("div",{className:T+"__container"},n.default.createElement("div",{className:T+"__content"},n.default.createElement("div",{className:T+"__title"},N),n.default.createElement(f.InviteModalInviteContent,{baseClass:T,focusRef:this.focusRef,currentUser:O,handleOnSubmit:h,onCSVImport:M,handleCSVImport:C,renderInputForm:w,renderSubmitBtn:E,showImportCSV:k,isInviteLinkEnabled:A,growthActSmbMWInvitesActive:L}),n.default.createElement(v.InviteModalMemberList,{baseClass:T,currentUserId:W,teamId:V,teamName:B,tieredAdmin:a,memberList:l,memberActionsData:d,handleOnClose:I,loggingExtras:S,growthActSmbMWInvitesActive:L})),u&&n.default.createElement(g.InviteModalSidebar,{baseClass:T,memberList:l,emails:_,sentEmails:p}))))},t})(n.default.Component);t.FirstTaskInviteModalWithoutCss=p,t.FirstTaskInviteModal=d.requireCssWithComponent(p,["/static/css/dig-components/index.web-vflZHG-C1.css","/static/css/teams/admin/widgets/invite_modal/invite_modal_first_task-vfluVBY-r.css"])}));
//# sourceMappingURL=invite_modal_first_task.min.js-vflQyrAcf.map