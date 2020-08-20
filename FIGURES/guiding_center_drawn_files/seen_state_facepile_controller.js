define(["require","exports","tslib","react","modules/clean/react/pass/seen_state_facepile_consumer","modules/clean/file_store/utils","modules/clean/react/pass/constants","modules/clean/react/pass/pass_helpers","modules/clean/react/pass/seen_state_helpers","modules/clean/react/pass/store","modules/clean/react/pass/types","modules/clean/react/pass/tooltip_helpers","modules/clean/sharing/stores/sharing_info","modules/core/i18n","modules/clean/react/pass/seen_state_facepile_context","modules/clean/react/pass/integration/integration_provider","modules/clean/react/teams/team_discovery/data/store","modules/clean/react/teams/team_discovery/data/selector","modules/clean/react/teams/team_discovery/data/client","modules/clean/react/teams/team_discovery/data/logger","modules/clean/api_v2/user_client","modules/clean/react/teams/team_discovery/data/action","modules/clean/react/teams/team_discovery/data/logger"],(function(e,t,s,n,a,o,r,i,l,p,u,c,m,f,d,h,g,S,_,I,P,v,y){"use strict";Object.defineProperty(t,"__esModule",{value:!0}),n=s.__importDefault(n);var F=g.getTeamDiscoveryStore();t.teamDiscoverySelector=new S.Selector,t.teamDiscoveryActionCreator=new v.ActionCreator((new P.UserApiV2Client).ns("team_discovery"),new _.LegacyClient,new I.Logger,t.teamDiscoverySelector);var D=(function(e){function a(t){var s=e.call(this,t)||this;return s.onUpdateFromStore=function(){s.setState(s.getStateFromStore())},s.state=s.getStateFromStore(),s}return s.__extends(a,e),a.prototype.componentWillMount=function(){var e=this.props,s=e.user,n=e.isViewMetadataDisabled;p.passStore.add_change_listener(this.onUpdateFromStore),m.sharingInfoStore.add_change_listener(this.onUpdateFromStore),F.subscribe(this.onUpdateFromStore),i.PassHelpers.setup({user:s,fileData:this.getPassFileData(),skipSharingEndpoints:n,prevFileData:null}),s&&c.getOnVsOffTeamExperiment()&&F.dispatch(t.teamDiscoveryActionCreator.fetchInitialData({userId:s.id,userEmail:s.email},100,!1))},a.prototype.componentWillReceiveProps=function(e){o.areFilesEqual(e.file,this.props.file)||this.setState({passInfo:{anonymousPresence:null,isPassPermissionsPending:!0,passFetchingStatus:r.PassFetchingStatus.FETCHING,passPermissions:null,presence:null,identifiedSeenStateInfo:null,userIdsWithoutSeenStateInfo:[]},sharingInfo:void 0})},a.prototype.componentWillUpdate=function(e,t){var s=t.passInfo;if(s.userIdsWithoutSeenStateInfo.length>0&&r.fetchingStatusIsSuccessful(s.passFetchingStatus)&&e.user){var n=e.file,a=this.urlFromProps(e);l.SeenStateHelpers.fetchSeenStateUsers(e.user.id,s.identifiedSeenStateInfo,s.userIdsWithoutSeenStateInfo,n.file_id,a)}},a.prototype.componentDidUpdate=function(e){if(!o.areFilesEqual(this.props.file,e.file)){var t={file:e.file,url:this.urlFromProps(e)},s=this.props.isViewMetadataDisabled;i.PassHelpers.setup({user:this.props.user,fileData:this.getPassFileData(),skipSharingEndpoints:s,prevFileData:t})}},a.prototype.componentWillUnmount=function(){p.passStore.remove_change_listener(this.onUpdateFromStore),m.sharingInfoStore.remove_change_listener(this.onUpdateFromStore),i.PassHelpers.teardown({user:this.props.user,fileData:this.getPassFileData(),async:!0})},a.prototype.getStateFromStore=function(){var e=this.props,t=e.user,s=e.file.file_id;return{passInfo:{anonymousPresence:p.passStore.anonymousPresence(s),isPassPermissionsPending:p.passStore.isPassPermissionsPending(s),passFetchingStatus:p.passStore.passFetchingStatus(s),passPermissions:p.passStore.getPassPermissions(s),presence:p.passStore.presence(s),identifiedSeenStateInfo:p.passStore.identifiedSeenStateInfo(s),userIdsWithoutSeenStateInfo:p.passStore.userIdsWithoutSeenStateInfo(s)},sharingInfo:t&&m.sharingInfoStore.getSharingInfo(t.id,s)||void 0,teamExpandInfo:this.getTeamExpandInfo()}},a.prototype.getTeamExpandInfo=function(){return{joinableTeams:t.teamDiscoverySelector.getTeamDiscoveryStoreState(F.getState()).joinableTeams,joinTeamHandler:function(e){F.dispatch(t.teamDiscoveryActionCreator.joinTeamAction(e,y.TeamJoinRequestOrigin.Pass,!1))}}},a.prototype.getAnonymousPassInfo=function(){return null==this.state.passInfo.anonymousPresence?[]:this.state.passInfo.anonymousPresence.map((function(e){return{seen_state_user:{user_id:e,display_name:f.intl.formatMessage({id:"+s6D7j",defaultMessage:"Guest"})},seen_events:[]}}))},a.prototype.getMemberInfosInPassFormat=function(e){var t=e.members();return{invitees:t?this.getInviteeMemberInfos(t):[],users:t?this.getUserMemberInfos(t):[]}},a.prototype.getUserMemberInfos=function(e){return e.users.valueSeq().toArray().reduce((function(e,t){return t.account&&e.push({seen_state_user:{user_id:t.account_id,display_name:t.account.display_name,email:t.email(),photo_circle_url:t.account.profile_photo_url||void 0,access_level:t.access_type,same_team:t.same_team},seen_events:[]}),e}),[])},a.prototype.getInviteeMemberInfos=function(e){return e.invitees.valueSeq().toArray().map((function(e){return{seen_state_user:{display_name:e.contact,access_level:e.access_type},seen_events:[]}}))},a.prototype.getPassFileData=function(){return{file:this.props.file,url:this.urlFromProps()}},a.prototype.urlFromProps=function(e){return void 0===e&&(e=this.props),e.sharedLinkInfo?e.sharedLinkInfo.url:void 0},a.prototype.getPropsFromPassInfo=function(){return{anonymousPresenceInfo:this.getAnonymousPassInfo(),isPassPermissionsPending:this.state.passInfo.isPassPermissionsPending,passFetchingStatus:this.state.passInfo.passFetchingStatus,passPermissions:this.state.passInfo.passPermissions,presence:this.state.passInfo.presence,identifiedSeenStateInfo:this.state.passInfo.identifiedSeenStateInfo}},a.prototype.getPropsFromSharingInfo=function(){var e={uniqueMemberCountInfo:u.FacepileInfo.createNotLoaded(),sharingStorePassInfo:u.FacepileInfo.createNotLoaded()};return this.state.sharingInfo?(null!=this.state.sharingInfo.memberCounts()&&(e.uniqueMemberCountInfo=u.FacepileInfo.create(this.state.sharingInfo.memberCounts().total_unique_users)),this.state.sharingInfo.hasDisplayableMembers()&&(e.sharingStorePassInfo=u.FacepileInfo.create(this.getMemberInfosInPassFormat(this.state.sharingInfo))),e):e},a.prototype.getPropsFromTeamDiscovery=function(){return this.state.teamExpandInfo||{}},a.prototype.getPropsFromController=function(){var e=this.props;return{file:e.file,sizeClass:e.sizeClass,url:this.urlFromProps()}},a.prototype.render=function(){var e=this.props,t=e.children,a=e.isViewingFileSubpath,o=e.user,r=this.getPropsFromPassInfo();return null==o||a?n.default.createElement(d.SeenStateFacepileContext.Provider,{value:{presenceComplete:!r.isPassPermissionsPending}},t):n.default.createElement(d.SeenStateFacepileContext.Provider,{value:s.__assign(s.__assign(s.__assign(s.__assign(s.__assign({},r),this.getPropsFromSharingInfo()),this.getPropsFromController()),this.getPropsFromTeamDiscovery()),{user:o})},n.default.createElement(h.IntegrationProvider,{user:o},t))},a})(n.default.Component);t.SeenStateFacepileProvider=D;var b=(function(e){function t(){return null!==e&&e.apply(this,arguments)||this}return s.__extends(t,e),t.prototype.render=function(){return n.default.createElement(D,s.__assign({},this.props),n.default.createElement(a.SeenStateFacepileConsumer,null))},t})(n.default.Component);t.SeenStateFacepileController=b}));
//# sourceMappingURL=seen_state_facepile_controller.min.js-vflISVtY9.map