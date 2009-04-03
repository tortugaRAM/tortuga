//
//  main.m
//  polo
//
//  Created by auvsi on 4/2/09.
//  Copyright __MyCompanyName__ 2009. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <Ice/Ice.h>
#import "AdapterContainer.h"

int main(int argc, char *argv[]) {
    
    NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
    
	int status = 1;
	id<ICECommunicator> communicator = nil;
	@try {
		communicator = [ICEUtil createCommunicator: &argc argv:argv];
		
		id<ICEObjectAdapter> adapter =
		[communicator createObjectAdapterWithEndpoints: @"PoloAdapter"
											 endpoints: @"default -p 10000"];
		[AdapterContainer setSharedAdapter: adapter];
		
		status = UIApplicationMain(argc, argv, nil, nil);
		
		[communicator waitForShutdown];
	} @catch (NSException* ex) {
		NSLog(@"%@", ex);
	}
	
	@try {
		[communicator destroy];
	} @catch (NSException* ex) {
		NSLog(@"%@", ex);
	}
	
    [pool release];
    return status;
}
